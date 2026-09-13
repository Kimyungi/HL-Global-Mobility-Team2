"""Fine-tune only COCO class 9 logits while keeping other inference weights fixed."""
from copy import deepcopy

import torch
from torch import nn
from ultralytics.models.yolo.detect import DetectionTrainer


class TrafficRowGuard:
    def __init__(self, model, class_id=9):
        head = model.model[-1]
        if head.nc != 80 or model.names[class_id] != 'traffic light':
            raise ValueError('Expected the original 80-class COCO head')
        self.class_id = class_id
        self.baseline = {k: v.detach().clone() for k, v in model.state_dict().items()}
        prefix = f'model.{len(model.model)-1}.cv3.'
        self.allowed = set()
        for scale, sequence in enumerate(head.cv3):
            final = sequence[-1]
            if not isinstance(final, nn.Conv2d) or final.out_channels != 80:
                raise ValueError('Unsupported YOLO classification head')
            for suffix in ('weight', 'bias'):
                self.allowed.add(f'{prefix}{scale}.{len(sequence)-1}.{suffix}')
        parameters = dict(model.named_parameters())
        if not self.allowed <= parameters.keys():
            raise ValueError('Could not locate classification output parameters')
        self.hooks = []
        for name, parameter in parameters.items():
            parameter.requires_grad_(name in self.allowed)
            if name in self.allowed:
                self.hooks.append(parameter.register_hook(self.mask_gradient))
        self.freeze_statistics(model)

    def mask_gradient(self, gradient):
        masked = torch.zeros_like(gradient)
        masked[self.class_id] = gradient[self.class_id]
        return masked

    @staticmethod
    def freeze_statistics(model):
        for module in model.modules():
            if isinstance(module, nn.modules.batchnorm._BatchNorm):
                module.eval()

    @torch.no_grad()
    def restore(self, model):
        # Also undo EMA rounding of unchanged weights and any optimizer decay
        # outside the allowed row. Never restore the learned traffic-light row.
        state = model.state_dict()
        for name, original in self.baseline.items():
            target = state[name]
            if name in self.allowed:
                target[:self.class_id].copy_(original[:self.class_id])
                target[self.class_id+1:].copy_(original[self.class_id+1:])
            else:
                target.copy_(original)

    def assert_preserved(self, model):
        state = model.state_dict()
        for name, original in self.baseline.items():
            target = state[name]
            expected = original.to(device=target.device, dtype=target.dtype)
            if name in self.allowed:
                equal = (torch.equal(target[:self.class_id], expected[:self.class_id])
                         and torch.equal(target[self.class_id+1:], expected[self.class_id+1:]))
            else:
                equal = torch.equal(target, expected)
            if not equal:
                raise RuntimeError('Frozen inference state changed: ' + name)


class PreserveTrafficTrainer(DetectionTrainer):
    def get_model(self, cfg=None, weights=None, verbose=True):
        # Rebuilding legacy YOLOv8 YAML with a newer Ultralytics can change
        # unparameterized operations (notably SPPF.cv1 SiLU -> Identity).
        # Copy the loaded module graph, including activations and attributes.
        if not isinstance(weights, nn.Module) or weights.model[-1].nc != 80:
            raise ValueError('An existing 80-class model module is required')
        return deepcopy(weights).float()

    def _setup_train(self):
        super()._setup_train()
        self.traffic_guard = TrafficRowGuard(self.model)
        self.traffic_guard.assert_preserved(self.ema.ema)

    def _model_train(self):
        super()._model_train()
        self.traffic_guard.freeze_statistics(self.model)

    def optimizer_step(self):
        super().optimizer_step()
        self.traffic_guard.restore(self.model)
        if self.ema:
            self.traffic_guard.restore(self.ema.ema)

    def save_model(self):
        self.traffic_guard.assert_preserved(self.model)
        self.traffic_guard.assert_preserved(self.ema.ema)
        return super().save_model()
