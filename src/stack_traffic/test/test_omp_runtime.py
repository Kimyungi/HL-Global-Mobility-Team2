"""OpenMP 스핀 억제가 실제로 걸리는지, 그리고 **순서**가 지켜지는지 잠근다.

순서 시험이 핵심이다 — 환경변수는 OpenMP 런타임 초기화 시점에 한 번만 읽히므로,
torch/ultralytics 를 먼저 import 하면 이 모듈은 아무 일도 하지 않는다. 그런데
그때도 예외 없이 조용히 통과하므로, 사람이 import 를 정리하다 순서를 바꿔도
눈치채지 못한다. 그래서 소스에서 순서를 검사한다.
"""
import os
import re
import unittest
from pathlib import Path

NODE = Path(__file__).parents[1] / "stack_traffic" / "node.py"


class TestOmpRuntime(unittest.TestCase):
    def test_import_sets_passive_wait(self):
        from stack_traffic import omp_runtime  # noqa: F401
        self.assertEqual(os.environ.get("KMP_BLOCKTIME"), "0")
        self.assertEqual(os.environ.get("OMP_WAIT_POLICY"), "PASSIVE")

    def test_does_not_override_explicit_value(self):
        """밖에서 준 값은 덮지 않는다 — 옛 거동으로 되돌릴 수 있어야 한다."""
        import importlib
        saved = os.environ.get("KMP_BLOCKTIME")
        os.environ["KMP_BLOCKTIME"] = "200"
        try:
            from stack_traffic import omp_runtime
            importlib.reload(omp_runtime)
            self.assertEqual(os.environ["KMP_BLOCKTIME"], "200")
        finally:
            if saved is None:
                del os.environ["KMP_BLOCKTIME"]
            else:
                os.environ["KMP_BLOCKTIME"] = saved

    def test_node_imports_omp_runtime_before_heavy_libs(self):
        source = NODE.read_text(encoding="utf-8")
        omp = source.find("from stack_traffic import omp_runtime")
        self.assertNotEqual(omp, -1, "node.py 가 omp_runtime 을 import 하지 않는다")
        for later in ("import cv2", "from ultralytics import YOLO",
                      "from stack_traffic.oak_camera import"):
            pos = source.find(later)
            self.assertNotEqual(pos, -1, f"{later!r} 를 찾지 못했다")
            self.assertLess(
                omp, pos,
                f"omp_runtime import 가 {later!r} 보다 뒤에 있다 — "
                "OpenMP 런타임이 이미 초기화되어 설정이 무시된다")

    def test_no_stray_env_writes_elsewhere(self):
        """설정 자리를 한 곳으로 묶어 둔다 (값이 두 곳으로 갈리는 사고 방지)."""
        pkg = NODE.parent
        for path in pkg.glob("*.py"):
            if path.name == "omp_runtime.py":
                continue
            text = path.read_text(encoding="utf-8")
            self.assertIsNone(
                re.search(r"environ\[[\"']KMP_BLOCKTIME", text),
                f"{path.name} 가 KMP_BLOCKTIME 을 따로 쓴다 — omp_runtime 로 모을 것")


if __name__ == "__main__":
    unittest.main()
