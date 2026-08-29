from contextlib import redirect_stdout
from io import StringIO
import importlib.metadata
import json
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from stack_traffic.ml_preflight import main, render_text, run_preflight


class FakeTensor:
    def numel(self):
        return 1


def missing_distribution(_name):
    raise importlib.metadata.PackageNotFoundError


class FakeDistribution:
    def __init__(self, version, root, requirements=()):
        self.version = version
        self.root = Path(root)
        self.requires = tuple(requirements)
        self.files = ()

    def locate_file(self, item):
        return self.root / item


class CorruptDistribution(FakeDistribution):
    def __init__(self, failing_attribute):
        object.__setattr__(self, "_failing_attribute", failing_attribute)
        super().__init__("2.12.0+xpu", "/opt/xpu", ("torch (==2.12.0)",))

    def __getattribute__(self, name):
        if name == object.__getattribute__(self, "_failing_attribute"):
            raise RuntimeError(f"corrupt metadata field: {name}")
        return object.__getattribute__(self, name)


def make_distributions(*, required_torch="2.12.0", vision_flavor="xpu"):
    distributions = {
        "torch": FakeDistribution("2.12.0+xpu", "/opt/xpu"),
        "torchvision": FakeDistribution(
            f"0.27.0+{vision_flavor}",
            "/opt/xpu",
            (f"torch (=={required_torch})",),
        ),
        "ultralytics": FakeDistribution("8.4.61", "/opt/xpu"),
        "depthai": FakeDistribution("3.6.1", "/opt"),
    }
    return distributions.__getitem__


def make_modules(*, xpu_available=True):
    torch = SimpleNamespace(
        __version__="2.12.0+xpu",
        __file__="/opt/xpu/torch/__init__.py",
        tensor=lambda _value: FakeTensor(),
        xpu=SimpleNamespace(is_available=lambda: xpu_available),
    )
    vision = SimpleNamespace(
        __version__="0.27.0+xpu",
        __file__="/opt/xpu/torchvision/__init__.py",
        extension=SimpleNamespace(_has_ops=lambda: True),
        ops=SimpleNamespace(
            nms=lambda _boxes, _scores, _threshold: FakeTensor()
        ),
    )
    ultralytics = SimpleNamespace(
        __version__="8.4.61",
        __file__="/opt/xpu/ultralytics/__init__.py",
        YOLO=lambda _path: None,
    )

    class Pipeline:
        def start(self):
            pass

    depthai = SimpleNamespace(
        __version__="3.6.1",
        __file__="/opt/depthai/__init__.py",
        Pipeline=Pipeline,
    )
    return {
        "torch": torch,
        "torchvision": vision,
        "ultralytics": ultralytics,
        "depthai": depthai,
    }


class TestMlPreflight(unittest.TestCase):
    def test_corrupt_distribution_fields_become_metadata_error(self):
        for failing_attribute in (
            "files",
            "version",
            "locate_file",
            "requires",
        ):
            with self.subTest(failing_attribute=failing_attribute):
                distributions = {
                    "torch": CorruptDistribution(failing_attribute),
                    "torchvision": FakeDistribution(
                        "0.27.0+xpu",
                        "/opt/xpu",
                        ("torch (==2.12.0)",),
                    ),
                    "ultralytics": FakeDistribution("8.4.61", "/opt/xpu"),
                    "depthai": FakeDistribution("3.6.1", "/opt"),
                }

                report = run_preflight(
                    import_fn=make_modules().__getitem__,
                    distribution_fn=distributions.__getitem__,
                )
                torch_info = next(
                    package
                    for package in report.packages
                    if package.name == "torch"
                )

                self.assertEqual(torch_info.metadata_version, "missing")
                self.assertIn(
                    "metadata_error=RuntimeError: "
                    f"corrupt metadata field: {failing_attribute}",
                    torch_info.metadata_root,
                )
                self.assertFalse(report.ready)
                self.assertIn("[FAIL] torch.metadata", render_text(report))
                json.dumps({"packages": [torch_info.__dict__]})

    def test_ready_with_working_nms_xpu_and_depthai3(self):
        modules = make_modules()
        report = run_preflight(
            require_xpu=True,
            import_fn=modules.__getitem__,
            distribution_fn=missing_distribution,
        )

        self.assertTrue(report.ready, render_text(report))
        self.assertIn("DepthAI 3.x API", render_text(report))
        self.assertIn("torchvision.ops.nms 실제 호출 성공", render_text(report))

    def test_supported_depthai2_runtime_passes(self):
        class V2Pipeline:
            pass

        modules = make_modules()
        modules["depthai"].Pipeline = V2Pipeline
        modules["depthai"].__version__ = "2.30.0"

        report = run_preflight(
            import_fn=modules.__getitem__,
            distribution_fn=missing_distribution,
        )

        self.assertTrue(report.ready, render_text(report))
        self.assertIn("DepthAI 2.x API", render_text(report))

    def test_torchvision_nms_registration_error_is_preserved(self):
        modules = make_modules()

        def broken_import(name):
            if name == "torchvision":
                raise RuntimeError("operator torchvision::nms does not exist")
            return modules[name]

        report = run_preflight(
            import_fn=broken_import,
            distribution_fn=missing_distribution,
        )

        self.assertFalse(report.ready)
        self.assertIn(
            "operator torchvision::nms does not exist",
            render_text(report),
        )
        self.assertIn("자동 설치는 수행하지 않았습니다", render_text(report))

    def test_xpu_is_optional_on_development_pc(self):
        modules = make_modules(xpu_available=False)
        report = run_preflight(
            import_fn=modules.__getitem__,
            distribution_fn=missing_distribution,
        )

        self.assertTrue(report.ready, render_text(report))
        self.assertIn("[WARN] torch.xpu", render_text(report))

    def test_xpu_is_required_when_requested(self):
        modules = make_modules(xpu_available=False)
        report = run_preflight(
            require_xpu=True,
            import_fn=modules.__getitem__,
            distribution_fn=missing_distribution,
        )

        self.assertFalse(report.ready)
        self.assertIn("[FAIL] torch.xpu", render_text(report))

    def test_torch_import_failure_blocks_vision_and_ultralytics(self):
        modules = make_modules()

        def broken_import(name):
            if name == "torch":
                raise OSError("libtorch.so not found")
            return modules[name]

        report = run_preflight(
            import_fn=broken_import,
            distribution_fn=missing_distribution,
        )
        text = render_text(report)

        self.assertFalse(report.ready)
        self.assertIn("libtorch.so not found", text)
        self.assertIn("torch/torchvision 실패로 ultralytics", text)

    def test_torchvision_exact_torch_requirement_mismatch_fails(self):
        report = run_preflight(
            import_fn=make_modules().__getitem__,
            distribution_fn=make_distributions(required_torch="2.11.0"),
        )

        self.assertFalse(report.ready)
        self.assertIn(
            "expected=2.11.0, actual=2.12.0",
            render_text(report),
        )

    def test_private_has_ops_absence_does_not_override_working_nms(self):
        modules = make_modules()
        del modules["torchvision"].extension

        report = run_preflight(
            import_fn=modules.__getitem__,
            distribution_fn=missing_distribution,
        )

        self.assertTrue(report.ready, render_text(report))
        self.assertIn("[WARN] torchvision.native_ops", render_text(report))
        self.assertIn("[PASS] torchvision.nms", render_text(report))

    def test_build_flavor_mismatch_fails(self):
        modules = make_modules()
        modules["torchvision"].__version__ = "0.27.0+cpu"

        report = run_preflight(
            import_fn=modules.__getitem__,
            distribution_fn=missing_distribution,
        )

        self.assertFalse(report.ready)
        self.assertIn("xpu != cpu", render_text(report))

    def test_mixed_torch_and_vision_install_roots_fail(self):
        distributions = {
            "torch": FakeDistribution("2.12.0+xpu", "/usr/lib/python3"),
            "torchvision": FakeDistribution(
                "0.27.0+xpu",
                "/home/user/.local/lib/python3",
                ("torch (==2.12.0)",),
            ),
            "ultralytics": FakeDistribution("8.4.61", "/opt/xpu"),
            "depthai": FakeDistribution("3.6.1", "/opt"),
        }

        report = run_preflight(
            import_fn=make_modules().__getitem__,
            distribution_fn=distributions.__getitem__,
        )

        self.assertFalse(report.ready)
        self.assertIn("torchvision.mixed_install_roots", render_text(report))

    def test_unsupported_early_depthai3_fails(self):
        modules = make_modules()
        modules["depthai"].__version__ = "3.5.0"

        report = run_preflight(
            import_fn=modules.__getitem__,
            distribution_fn=missing_distribution,
        )

        self.assertFalse(report.ready)
        self.assertIn("runtime=3.5.0", render_text(report))
        self.assertIn("지원 범위는 2.30+ 또는 3.6+", render_text(report))

    def test_depthai_runtime_and_detected_api_major_must_match(self):
        class V2Pipeline:
            pass

        mismatches = (
            ("4.0.0", None, "DepthAI 3.x API"),
            ("3.6.1", V2Pipeline, "DepthAI 2.x API"),
            ("2.30.0", None, "DepthAI 3.x API"),
        )
        for version, pipeline, expected_api in mismatches:
            with self.subTest(version=version, expected_api=expected_api):
                modules = make_modules()
                modules["depthai"].__version__ = version
                if pipeline is not None:
                    modules["depthai"].Pipeline = pipeline

                report = run_preflight(
                    import_fn=modules.__getitem__,
                    distribution_fn=missing_distribution,
                )

                rendered = render_text(report)
                self.assertFalse(report.ready)
                self.assertIn(expected_api, rendered)
                self.assertIn(
                    "runtime/API major 일치 필수",
                    rendered,
                )

    def test_json_mode_keeps_import_chatter_inside_valid_document(self):
        report = run_preflight(
            import_fn=make_modules().__getitem__,
            distribution_fn=missing_distribution,
        )

        def noisy_preflight(**_kwargs):
            print("Creating new Ultralytics Settings...")
            os.write(1, b"native depthai warning\n")
            return report

        output = StringIO()
        with (
            patch(
                "stack_traffic.ml_preflight.run_preflight",
                side_effect=noisy_preflight,
            ),
            redirect_stdout(output),
        ):
            exit_code = main(["--json"])

        payload = json.loads(output.getvalue())
        self.assertEqual(exit_code, 0)
        self.assertTrue(payload["ready"])
        self.assertIn(
            "Creating new Ultralytics Settings...",
            payload["captured_import_output"],
        )
        self.assertIn(
            "native depthai warning",
            payload["captured_import_output"],
        )


if __name__ == "__main__":
    unittest.main()
