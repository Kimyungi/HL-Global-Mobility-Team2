"""USB 장치 리셋 — 로버 GNSS가 응답을 멈췄을 때 "뺐다 꽂기"의 소프트웨어 등가물.

**왜 필요한가 (2026-08-18 실차):** 한 코스 주행 중 3회 재시작했는데, 그 중
두 번은 재시작 직후 GPS가 아예 죽어 있었다. 증상이 특이하다:

  · RTCM **쓰기는 성공**한다 (stack_gps 로그: `fix 없음 (RTCM 581B/s)` 가 계속)
  · NMEA 는 **한 줄도 안 온다** (fix 없음이 52초 내내)
  · `rtk_probe.py` 로 재면 C/N0 **0dB** — 위성이 하나도 안 보인다
  · USB 를 뽑았다 꽂으면 즉시 복구

즉 장치는 열거된 채로 남아 있는데(그래서 write 는 성공) 수신기 쪽 출력이
멈춘 상태다. 포트를 닫았다 다시 여는 것으로는 안 풀린다 — USB 레벨에서
재열거를 시켜야 한다. `USBDEVFS_RESET` 이 그것이고, 커널이 포트 리셋 후
드라이버를 다시 바인딩한다(= 뽑았다 꽂기와 같은 경로, 전원만 유지).

⚠ 전원을 끊는 것은 아니다. 수신기 펌웨어가 완전히 걸려 전원 재인가가 필요한
   상태라면 이것으로 안 풀릴 수 있다 — 그때는 로그에 남는 실패를 보고 사람이
   물리적으로 뽑았다 꽂아야 한다. 실측상 어느 쪽인지 아직 확정되지 않았다.

**권한:** `/dev/bus/usb/<bus>/<dev>` 에 쓰기가 필요하다. 기본 권한은
root:root 0664 라 그냥은 안 된다 — `tools/99-ublox-f9p-usbreset.rules` 를
설치하면 이 장치에 한해 0666 이 되어 sudo 없이 리셋할 수 있다.
"""
import fcntl
import os
import time

# linux/usbdevice_fs.h: #define USBDEVFS_RESET _IO('U', 20)
USBDEVFS_RESET = ord('U') << 8 | 20


def usb_device_path(tty_path):
    """/dev/ttyRover(심볼릭 링크 포함) → (/dev/bus/usb/BBB/DDD, sysfs 경로).

    tty → 인터페이스 → usb_device 로 sysfs 를 거슬러 올라가 busnum/devnum 을 읽는다.
    (경로를 문자열로 조립하지 않는 이유: 허브 구성이 바뀌면 3-4 같은 이름이 바뀐다)
    """
    name = os.path.basename(os.path.realpath(tty_path))     # ttyACM0
    dev = f"/sys/class/tty/{name}/device"                   # …/3-4:1.0
    if not os.path.exists(dev):
        raise FileNotFoundError(f"sysfs 에서 {name} 을 찾을 수 없다")
    node = os.path.realpath(dev)
    for _ in range(4):                                      # 인터페이스 → 장치
        if os.path.exists(os.path.join(node, "busnum")):
            break
        node = os.path.dirname(node)
    else:
        raise FileNotFoundError(f"{name} 의 usb_device 부모를 못 찾았다")
    with open(os.path.join(node, "busnum")) as f:
        bus = int(f.read())
    with open(os.path.join(node, "devnum")) as f:
        num = int(f.read())
    return f"/dev/bus/usb/{bus:03d}/{num:03d}", node


def reset(tty_path, log=print):
    """tty 뒤의 USB 장치를 리셋한다. 성공 True / 실패 False (이유는 log 로).

    호출 전에 시리얼 포트를 **닫아 두어야** 한다 — 열린 채로 리셋하면 커널이
    드라이버를 떼면서 그 핸들이 무효가 되고, 이후 read 가 EIO 로 떨어진다.
    """
    try:
        usbfs, sysfs = usb_device_path(tty_path)
    except (OSError, FileNotFoundError) as e:
        log(f"USB 리셋 실패 — 장치 경로를 못 찾음: {e}")
        return False
    try:
        fd = os.open(usbfs, os.O_WRONLY)
    except PermissionError:
        log(f"USB 리셋 실패 — {usbfs} 쓰기 권한 없음. 아래 한 줄로 해결된다:\n"
            "  sudo cp ~/FMA_ws/src/stack_gps/tools/99-ublox-f9p-usbreset.rules "
            "/etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger")
        return False
    except OSError as e:
        log(f"USB 리셋 실패 — {usbfs} 열기 오류: {e}")
        return False
    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
        log(f"USB 리셋 실행: {usbfs} ({os.path.basename(sysfs)})")
        return True
    except OSError as e:
        log(f"USB 리셋 실패 — ioctl 오류: {e}")
        return False
    finally:
        os.close(fd)


def wait_for_tty(tty_path, timeout=15.0, log=print):
    """리셋 후 장치 노드가 다시 나타날 때까지 기다린다.

    재열거 + udev 규칙 적용(심볼릭 링크 재생성)에 보통 1~3초 걸린다.
    노드가 살아난 직후엔 아직 열리지 않는 순간이 있어, 열어 보는 것까지 확인한다.
    """
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if os.path.exists(tty_path):
            try:
                fd = os.open(tty_path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                os.close(fd)
                waited = timeout - (end - time.monotonic())
                log(f"USB 재열거 완료 ({waited:.1f}s) — {tty_path}")
                return True
            except OSError:
                pass
        time.sleep(0.2)
    log(f"USB 재열거 대기 시간 초과 ({timeout:.0f}s) — {tty_path} 가 돌아오지 않았다. "
        "수신기 전원 재인가가 필요할 수 있다 (물리적으로 뽑았다 꽂기)")
    return False
