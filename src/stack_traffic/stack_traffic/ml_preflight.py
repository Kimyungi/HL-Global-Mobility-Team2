"""stack_traffic ML/DepthAI 런타임을 변경 없이 진단한다."""

from __future__ import annotations

import argparse
from contextlib import redirect_stdout
from dataclasses import asdict, dataclass
from io import StringIO
import importlib
from importlib import metadata
import json
import os
from pathlib import Path
import re
import sys
import tempfile
from typing import Callable, Optional


@dataclass(frozen=True)
class PackageInfo:
    """설치 metadata와 실제 import 위치."""

    name: str
    metadata_version: str = "missing"
    metadata_root: str = "unknown"
    module_version: str = "unknown"
    module_file: str = "unknown"
    native_extension: str = "unknown"
    requirements: tuple[str, ...] = ()


@dataclass(frozen=True)
class CheckResult:
    """한 진단 항목의 결과."""

    code: str
    status: str
    message: str


@dataclass(frozen=True)
class MlRuntimeReport:
    """현장 로그에 그대로 저장할 수 있는 진단 보고서."""

    python_executable: str
    python_version: str
    packages: tuple[PackageInfo, ...]
    checks: tuple[CheckResult, ...]

    @property
    def ready(self) -> bool:
        return not any(check.status == "FAIL" for check in self.checks)


def _distribution_info(name: str, distribution_fn) -> PackageInfo:
    try:
        distribution = distribution_fn(name)
    except metadata.PackageNotFoundError:
        return PackageInfo(name=name)
    except Exception as error:
        return PackageInfo(
            name=name,
            metadata_root=(
                f"metadata_error={type(error).__name__}: {error}"
            ),
        )

    try:
        native_extension = "unknown"
        for item in distribution.files or ():
            text = str(item)
            if name == "torchvision" and re.search(
                r"torchvision/_C[^/]*\.so$",
                text,
            ):
                native_extension = str(distribution.locate_file(item))
                break
        return PackageInfo(
            name=name,
            metadata_version=distribution.version,
            metadata_root=str(distribution.locate_file("")),
            native_extension=native_extension,
            requirements=tuple(distribution.requires or ()),
        )
    except Exception as error:
        return PackageInfo(
            name=name,
            metadata_root=(
                f"metadata_error={type(error).__name__}: {error}"
            ),
        )


def _with_module(info: PackageInfo, module) -> PackageInfo:
    return PackageInfo(
        name=info.name,
        metadata_version=info.metadata_version,
        metadata_root=info.metadata_root,
        module_version=str(getattr(module, "__version__", "unknown")),
        module_file=str(getattr(module, "__file__", "unknown")),
        native_extension=info.native_extension,
        requirements=info.requirements,
    )


def _build_flavor(version: str) -> Optional[str]:
    if "+" not in version:
        return None
    return version.split("+", 1)[1].lower()


def _base_version(version: str) -> str:
    return version.split("+", 1)[0].strip()


def _depthai_version_supported(version: str, api_major: int) -> bool:
    numbers = tuple(int(value) for value in re.findall(r"\d+", version))
    minimum = {2: (2, 30), 3: (3, 6)}.get(api_major)
    return (
        minimum is not None
        and bool(numbers)
        and numbers[0] == api_major
        and numbers[:len(minimum)] >= minimum
    )


def _required_torch_version(vision_info: PackageInfo) -> Optional[str]:
    for requirement in vision_info.requirements:
        match = re.match(
            r"^torch\s*\(\s*==\s*([^;)\s]+)\s*\)",
            requirement,
            flags=re.IGNORECASE,
        )
        if match:
            return match.group(1)
        match = re.match(
            r"^torch\s*==\s*([^;\s]+)",
            requirement,
            flags=re.IGNORECASE,
        )
        if match:
            return match.group(1)
    return None


def _metadata_runtime_checks(
    package_map: dict[str, PackageInfo],
) -> list[CheckResult]:
    checks = []
    for name, info in package_map.items():
        if info.module_version == "unknown":
            continue
        if info.metadata_root.startswith("metadata_error="):
            checks.append(
                CheckResult(
                    f"{name}.metadata",
                    "FAIL",
                    "설치 metadata를 읽을 수 없습니다: "
                    f"{info.metadata_root}",
                )
            )
            continue
        if info.metadata_version == "missing":
            checks.append(
                CheckResult(
                    f"{name}.metadata",
                    "WARN",
                    "import는 성공했지만 설치 metadata를 찾지 못했습니다.",
                )
            )
            continue
        if _base_version(info.metadata_version) != _base_version(
            info.module_version
        ):
            checks.append(
                CheckResult(
                    f"{name}.metadata_version",
                    "FAIL",
                    "metadata/runtime 버전이 다릅니다: "
                    f"{info.metadata_version} != {info.module_version}",
                )
            )
        if (
            info.metadata_root not in ("unknown", "missing")
            and not info.metadata_root.startswith("metadata_error=")
            and info.module_file != "unknown"
        ):
            try:
                module_path = Path(info.module_file).resolve()
                metadata_root = Path(info.metadata_root).resolve()
                module_path.relative_to(metadata_root)
            except (OSError, ValueError):
                checks.append(
                    CheckResult(
                        f"{name}.install_path",
                        "FAIL",
                        "metadata와 실제 import 경로가 다릅니다: "
                        f"root={info.metadata_root}, "
                        f"module={info.module_file}",
                    )
                )
    torch_root = package_map["torch"].metadata_root
    vision_root = package_map["torchvision"].metadata_root
    comparable_roots = (torch_root, vision_root)
    if all(
        root not in ("unknown", "missing")
        and not root.startswith("metadata_error=")
        for root in comparable_roots
    ):
        try:
            roots_match = (
                Path(torch_root).resolve() == Path(vision_root).resolve()
            )
        except OSError:
            roots_match = False
        if not roots_match:
            checks.append(
                CheckResult(
                    "torchvision.mixed_install_roots",
                    "FAIL",
                    "torch와 torchvision 설치 root가 다릅니다: "
                    f"torch={torch_root}, torchvision={vision_root}",
                )
            )
    return checks


def run_preflight(
    *,
    require_xpu: bool = False,
    import_fn: Callable[[str], object] = importlib.import_module,
    distribution_fn=metadata.distribution,
) -> MlRuntimeReport:
    """패키지를 설치·수정하지 않고 import/NMS/XPU/API를 확인한다."""
    package_names = ("torch", "torchvision", "ultralytics", "depthai")
    package_map = {
        name: _distribution_info(name, distribution_fn)
        for name in package_names
    }
    checks: list[CheckResult] = []
    try:
        torch = import_fn("torch")
        package_map["torch"] = _with_module(package_map["torch"], torch)
        checks.append(CheckResult("torch.import", "PASS", "torch import 성공"))
    except Exception as error:
        checks.append(
            CheckResult(
                "torch.import",
                "FAIL",
                f"{type(error).__name__}: {error}",
            )
        )
        torch = None

    if torch is not None:
        expected_torch = _required_torch_version(package_map["torchvision"])
        if expected_torch is not None:
            actual_torch = _base_version(package_map["torch"].module_version)
            checks.append(
                CheckResult(
                    "torchvision.torch_requirement",
                    "PASS" if actual_torch == expected_torch else "FAIL",
                    "torchvision 요구 torch 버전: "
                    f"expected={expected_torch}, actual={actual_torch}",
                )
            )

    vision = None
    if torch is None:
        checks.append(
            CheckResult(
                "torchvision.import",
                "FAIL",
                "torch import 실패로 torchvision 검사를 진행할 수 없음",
            )
        )
    else:
        try:
            vision = import_fn("torchvision")
            package_map["torchvision"] = _with_module(
                package_map["torchvision"],
                vision,
            )
            checks.append(
                CheckResult(
                    "torchvision.import",
                    "PASS",
                    "torchvision import 성공",
                )
            )
        except Exception as error:
            checks.append(
                CheckResult(
                    "torchvision.import",
                    "FAIL",
                    f"{type(error).__name__}: {error}",
                )
            )

    if torch is not None and vision is not None:
        try:
            has_ops = bool(vision.extension._has_ops())
        except Exception as error:
            has_ops = None
            ops_message = (
                "private _has_ops 확인 불가(실제 NMS 결과를 기준으로 판정): "
                f"{type(error).__name__}: {error}"
            )
        else:
            ops_message = (
                "torchvision native op 로드 성공"
                if has_ops
                else "torchvision.extension._has_ops()=False"
            )

        try:
            boxes = torch.tensor(
                [[0.0, 0.0, 10.0, 10.0], [1.0, 1.0, 9.0, 9.0]]
            )
            scores = torch.tensor([0.9, 0.8])
            kept = vision.ops.nms(boxes, scores, 0.5)
            if int(kept.numel()) < 1:
                raise RuntimeError("NMS가 빈 결과를 반환했습니다.")
        except Exception as error:
            nms_passed = False
            checks.append(
                CheckResult(
                    "torchvision.nms",
                    "FAIL",
                    f"{type(error).__name__}: {error}",
                )
            )
        else:
            nms_passed = True
            checks.append(
                CheckResult(
                    "torchvision.nms",
                    "PASS",
                    "torchvision.ops.nms 실제 호출 성공",
                )
            )

        checks.append(
            CheckResult(
                "torchvision.native_ops",
                "PASS" if has_ops else (
                    "FAIL" if has_ops is False and not nms_passed else "WARN"
                ),
                ops_message,
            )
        )

        torch_version = package_map["torch"].module_version
        vision_version = package_map["torchvision"].module_version
        torch_flavor = _build_flavor(torch_version)
        vision_flavor = _build_flavor(vision_version)
        if (
            torch_flavor is not None
            and vision_flavor is not None
            and torch_flavor != vision_flavor
        ):
            checks.append(
                CheckResult(
                    "torch.build_flavor",
                    "FAIL",
                    "torch/torchvision wheel 채널이 다릅니다: "
                    f"{torch_flavor} != {vision_flavor}",
                )
            )

    if torch is not None:
        try:
            xpu_available = bool(
                hasattr(torch, "xpu") and torch.xpu.is_available()
            )
        except Exception as error:
            xpu_available = False
            xpu_message = f"XPU 확인 실패: {type(error).__name__}: {error}"
        else:
            xpu_message = f"torch.xpu.is_available()={xpu_available}"
        checks.append(
            CheckResult(
                "torch.xpu",
                "PASS" if xpu_available else (
                    "FAIL" if require_xpu else "WARN"
                ),
                xpu_message,
            )
        )

    if torch is None or vision is None:
        checks.append(
            CheckResult(
                "ultralytics.import",
                "FAIL",
                "torch/torchvision 실패로 ultralytics 검사를 차단함",
            )
        )
    else:
        try:
            ultralytics = import_fn("ultralytics")
            package_map["ultralytics"] = _with_module(
                package_map["ultralytics"],
                ultralytics,
            )
            if not callable(getattr(ultralytics, "YOLO", None)):
                raise RuntimeError("ultralytics.YOLO가 callable이 아닙니다.")
        except Exception as error:
            checks.append(
                CheckResult(
                    "ultralytics.import",
                    "FAIL",
                    f"{type(error).__name__}: {error}",
                )
            )
        else:
            checks.append(
                CheckResult(
                    "ultralytics.import",
                    "PASS",
                    "ultralytics.YOLO import 성공",
                )
            )

    try:
        depthai = import_fn("depthai")
        package_map["depthai"] = _with_module(
            package_map["depthai"],
            depthai,
        )
        api_major = 3 if hasattr(depthai.Pipeline, "start") else 2
        depthai_version = package_map["depthai"].module_version
    except Exception as error:
        checks.append(
            CheckResult(
                "depthai.import",
                "FAIL",
                f"{type(error).__name__}: {error}",
            )
        )
    else:
        checks.append(
            CheckResult(
                "depthai.import",
                (
                    "PASS"
                    if _depthai_version_supported(
                        depthai_version,
                        api_major,
                    )
                    else "FAIL"
                ),
                f"DepthAI {api_major}.x API 감지, runtime={depthai_version}; "
                "지원 범위는 2.30+ 또는 3.6+, "
                "runtime/API major 일치 필수",
            )
        )

    checks.extend(_metadata_runtime_checks(package_map))

    return MlRuntimeReport(
        python_executable=sys.executable,
        python_version=sys.version.split()[0],
        packages=tuple(package_map[name] for name in package_names),
        checks=tuple(checks),
    )


def render_text(report: MlRuntimeReport) -> str:
    """터미널용 간결한 진단 결과를 만든다."""
    lines = [
        f"python={report.python_executable} ({report.python_version})",
    ]
    for package in report.packages:
        lines.append(
            f"{package.name} metadata={package.metadata_version} "
            f"runtime={package.module_version} "
            f"root={package.metadata_root} file={package.module_file}"
        )
        if package.native_extension != "unknown":
            lines.append(
                f"  native_extension={package.native_extension}"
            )
    for check in report.checks:
        lines.append(f"[{check.status}] {check.code}: {check.message}")
    lines.append(
        "ML_RUNTIME_READY" if report.ready else "ML_RUNTIME_NOT_READY"
    )
    if not report.ready:
        lines.append(
            "자동 설치는 수행하지 않았습니다. torch와 torchvision을 "
            "같은 공식 CPU/CUDA/XPU 채널의 호환 쌍으로 맞춘 뒤 "
            "재실행하세요."
        )
        extension = next(
            (
                package.native_extension
                for package in report.packages
                if package.name == "torchvision"
            ),
            "unknown",
        )
        if extension != "unknown" and Path(extension).exists():
            lines.append(f"공유 라이브러리 확인: ldd {extension}")
    return "\n".join(lines)


def _run_preflight_with_captured_stdout(
    *,
    require_xpu: bool,
) -> tuple[MlRuntimeReport, str]:
    """Python 및 native extension의 stdout을 JSON 밖으로 격리한다."""
    python_output = StringIO()
    sys.stdout.flush()
    saved_stdout_fd = os.dup(1)
    try:
        with tempfile.TemporaryFile(mode="w+b") as native_output:
            os.dup2(native_output.fileno(), 1)
            try:
                with redirect_stdout(python_output):
                    report = run_preflight(require_xpu=require_xpu)
            finally:
                sys.stdout.flush()
                os.dup2(saved_stdout_fd, 1)
            native_output.seek(0)
            native_text = native_output.read().decode(
                "utf-8",
                errors="replace",
            )
    finally:
        os.close(saved_stdout_fd)
    chatter = "\n".join(
        text.strip()
        for text in (python_output.getvalue(), native_text)
        if text.strip()
    )
    return report, chatter


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="stack_traffic ML/DepthAI 런타임 사전점검",
    )
    parser.add_argument(
        "--require-xpu",
        action="store_true",
        help="산업용 PC에서 Intel XPU 가용성을 필수로 검사",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="PR/현장 로그 첨부용 JSON 출력",
    )
    args = parser.parse_args(argv)
    previous_yolo_config = os.environ.get("YOLO_CONFIG_DIR")
    previous_matplotlib_config = os.environ.get("MPLCONFIGDIR")
    with tempfile.TemporaryDirectory(
        prefix="stack-traffic-preflight-",
        dir="/tmp",
    ) as config_dir:
        os.environ["YOLO_CONFIG_DIR"] = config_dir
        os.environ["MPLCONFIGDIR"] = config_dir
        try:
            if args.json:
                report, chatter = _run_preflight_with_captured_stdout(
                    require_xpu=args.require_xpu,
                )
            else:
                report = run_preflight(require_xpu=args.require_xpu)
        finally:
            if previous_yolo_config is None:
                os.environ.pop("YOLO_CONFIG_DIR", None)
            else:
                os.environ["YOLO_CONFIG_DIR"] = previous_yolo_config
            if previous_matplotlib_config is None:
                os.environ.pop("MPLCONFIGDIR", None)
            else:
                os.environ["MPLCONFIGDIR"] = previous_matplotlib_config
    if args.json:
        payload = asdict(report)
        payload["ready"] = report.ready
        if chatter:
            payload["captured_import_output"] = chatter
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    else:
        print(render_text(report))
    return 0 if report.ready else 1


if __name__ == "__main__":
    raise SystemExit(main())
