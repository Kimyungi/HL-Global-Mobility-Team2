from pathlib import Path
import copy
import torch
from ultralytics import YOLO
from preserve_traffic_head import TrafficRowGuard


def test_only_traffic_scores_can_change():
    torch.set_num_threads(2)
    root=Path(__file__).resolve().parents[2]
    model=YOLO(str(root/'src/stack_traffic/models/yolov8n.pt')).model.eval()
    image=torch.rand(1,3,128,128)
    with torch.no_grad(): before=model(image)[0].clone()
    guard=TrafficRowGuard(model)
    model.train();guard.freeze_statistics(model)
    with torch.no_grad(): model(image)
    guard.assert_preserved(model)
    optimizer=torch.optim.SGD([p for p in model.parameters() if p.requires_grad],lr=.001,weight_decay=.01)
    parameters=dict(model.named_parameters())
    for _ in range(3):
        optimizer.zero_grad()
        sum(parameters[name].sum() for name in guard.allowed).backward()
        for name in guard.allowed:
            grad=parameters[name].grad
            assert torch.count_nonzero(grad[:9])==0
            assert torch.count_nonzero(grad[10:])==0
            assert torch.count_nonzero(grad[9])>0
        optimizer.step();guard.restore(model);guard.assert_preserved(model)
    model.eval()
    with torch.no_grad(): after=model(image)[0]
    preserved_channels=[i for i in range(84) if i!=13]
    assert torch.equal(before[:,preserved_channels],after[:,preserved_channels])
    assert not torch.equal(before[:,13],after[:,13])
    ema=copy.deepcopy(model)
    with torch.no_grad():
        for p in ema.parameters(): p.mul_(.9999)
    guard.restore(ema);guard.assert_preserved(ema)


def test_trainer_keeps_legacy_activation_graph():
    from preserve_traffic_head import PreserveTrafficTrainer
    torch.set_num_threads(2)
    root=Path(__file__).resolve().parents[2]
    original=YOLO(str(root/'src/stack_traffic/models/yolov8n.pt')).model.eval()
    trainer=object.__new__(PreserveTrafficTrainer)
    copied=trainer.get_model(cfg=original.yaml,weights=original).eval()
    assert type(copied.model[9].cv1.act) is type(original.model[9].cv1.act)
    image=torch.rand(1,3,128,128)
    with torch.no_grad():
        assert torch.equal(original(image)[0],copied(image)[0])
