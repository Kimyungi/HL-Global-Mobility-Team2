# 이 PC의 Lunar Lake XPU 활성화

확인 환경: Core Ultra 9 288V, PCI 8086:64a0, Ubuntu 22.04.5,
6.8.0-138-generic, torch 2.12.1+xpu. 초기 상태는 xe가 force_probe를 요구하며
GPU에 바인딩되지 않았고 /dev/dri/renderD*가 없었다.

Intel 공식 표에서 64A0은 6.8 실험 지원, Ubuntu 24.04+ / 6.11 완전 지원이다.
https://dgpu-docs.intel.com/overview/supported-hardware/xe-driver-gpus.html
현재 ROS Humble/Jammy 사용자 공간을 유지하면서 Canonical Noble HWE의
`6.14.0-37-generic` 커널을 기존 6.8 옆에 시험 설치했다. `force_probe`는 쓰지 않는다.

## 2026-09-08: 부팅 실패로 설정 철회

force_probe 적용 후 사용자가 Ubuntu 로딩 중 멈춤을 보고했다.
GRUB에서 nomodeset을 일회 추가해 부팅했다. 실패한 부팅의 상세 로그는 확보하지
못했으므로 내부 실패 지점은 미확인이다. force_probe 설정 파일을 삭제하고
update-grub를 성공적으로 실행해 원래 quiet splash 설정으로 복구했다.
nomodeset은 영구 설정에 추가하지 않았다. 다음 정상 부팅 여부는 재확인이 필요하다.

`setup_lunar_lake.sh --apply`는 재발 방지를 위해 차단했다.

## 2026-09-09: 지원 커널 병렬 설치

`6.14.0-37-generic`의 image/modules/modules-extra를 설치하고 6.8.0-138은 복구용으로
유지했다. Lunar Lake용 `lnl_gsc_1.bin`, `lnl_guc_70.bin`, `lnl_huc.bin`,
`xe2lpd_dmc.bin`도 20250509 linux-firmware 태그에서 설치했다. 패키지와 펌웨어는
설치 전에 고정 SHA-256으로 검증했다. GRUB 메뉴는 10초간 항상 표시된다.

첫 6.14 부팅은 로딩 표시 후 화면이 나오지 않아 `nomodeset` 일회 편집으로 진입했다.
Ubuntu 22.04의 Mesa 23.2는 Lunar Lake 최소 요구 24.2보다 낮았다. Jammy용
kisak-mesa stable(turtle)의 Mesa 25.0.7/libdrm 2.4.122로 갱신했으며 패키지 삭제는
없었다. GNOME 42 Wayland 변수를 줄이기 위해 GDM은 우선 Xorg로 고정했다.
Intel 공식 Jammy GPU 저장소에서 Level Zero 로더 1.21.9와 GPU/OpenCL 런타임
25.18, `intel-ocloc`도 설치했다. 설치 전에는 `libze_loader.so`가 없어서 커널과
별개로 PyTorch XPU가 장치를 열 수 없는 상태였다.

재부팅 후 기본 6.14로 정상 진입하면 아래 검증 도구를 실행한다:
```bash
/usr/bin/python3 scripts/gpu/check_xpu.py
```
렌더 장치 접근 → torch XPU 실제 계산 → 기존 YOLOPv2 FP16 추론 순서로 검증한다.
모두 성공하면 USB 연결 후 별도로 카메라/ROS 포함 처리 주기를 확인한다.
검증 성공 후 2 m/s 카메라-GPS 실행은 XPU를 기본 사용한다:
```bash
./scripts/run_lane_gps_2mps.sh --start
```
필요할 때만 `LANE_DEVICE=cpu`로 CPU 폴백할 수 있다.

6.14 부팅에 실패하면 GRUB의 `Advanced options for Ubuntu`에서
`Ubuntu, with Linux 6.8.0-138-generic`을 선택한다. 여기서는 `nomodeset`이나
`force_probe`를 추가하지 않는다. 6.8로 진입한 뒤 시험 커널 제거:
```bash
sudo bash scripts/gpu/remove_test_kernel.sh
```
이 시험 커널은 Ubuntu 22.04용 정식 패키지 조합이 아니므로 XPU 검증을 위한 과도기
구성이다. 장기 운영 시에는 Ubuntu 24.04 이상과 그 배포판 커널로 환경 이전을 검토한다.

Mesa와 GDM 설정만 되돌리기:
```bash
sudo bash scripts/gpu/rollback_mesa_xorg.sh
```

실패 시 확인:
```bash
journalctl -k -b | rg -i 'xe|drm|firmware'
lspci -nnk -s 00:02.0
ls -l /dev/dri
id
```
render 장치가 생겨도 추론이 실패하면 다음 단계로 런타임 호환성을 조사한다.
XPU 검증 실패 시 force_probe를 반복하지 않고 6.8로 복구한다.
