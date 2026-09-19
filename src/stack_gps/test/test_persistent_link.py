"""Receiver observations survive route clients; quality is never latched to FIXED."""
from types import SimpleNamespace as NS

import pytest
from stack_gps import persistent_link as client
from stack_gps import persistent_service as service


def test_live_status_shows_rtcm_rate_and_actual_fix(monkeypatch):
    monkeypatch.setattr(service.time, 'monotonic', lambda: 102.)
    previous = dict(service_id='receiver', observed_at=100., rtcm_bytes=1000)
    current = dict(service_id='receiver', observed_at=102., rtcm_bytes=2120,
                   pid=123, fix=[37., 127., 5., 4, .1, 101.9])
    line = service.status_line(current, previous)
    assert '560 B/s | FIXED (FIX=4)' in line
    current['fix'][3] = 5
    assert 'FLOAT (FIX=5)' in service.status_line(current, previous)
    current['fix'][3] = 4
    current['fix'][5] = 100.
    assert 'NO FIX (FIX=0)' in service.status_line(current, previous)
    current['service_id'] = 'restarted'
    assert '-- B/s' in service.status_line(current, previous)


def test_start_monitors_until_ctrl_c_without_stopping_service(monkeypatch, capsys):
    current = dict(service_id='receiver', observed_at=100., rtcm_bytes=1000,
                   pid=123, fix=[37., 127., 5., 4, .1, 99.9])
    monkeypatch.setattr(service.time, 'monotonic', lambda: 100.)
    monkeypatch.setattr(service.sys, 'argv', ['gps', 'start', '--config', 'fake'])
    monkeypatch.setattr(service, 'configuration', lambda _: {})
    monkeypatch.setattr(service, 'ensure_running', lambda *args: current)
    calls = []
    def snapshot(op='snapshot', **kwargs):
        calls.append(op)
        return dict(current, observed_at=101., rtcm_bytes=1560)
    monkeypatch.setattr(service, 'request', snapshot)
    sleeps = []
    def sleep(_):
        sleeps.append(1)
        if len(sleeps) == 2:
            raise KeyboardInterrupt
    monkeypatch.setattr(service.time, 'sleep', sleep)
    service.main()
    output = capsys.readouterr().out
    assert '560 B/s | FIXED (FIX=4)' in output
    assert 'GPS/RTCM 연결은 계속 유지됩니다' in output
    assert calls == ['snapshot']


def test_monitor_service_loss_reports_disconnect(monkeypatch, capsys):
    monkeypatch.setattr(service.time, 'sleep', lambda _: None)
    monkeypatch.setattr(service, 'request', lambda **kw: (_ for _ in ()).throw(OSError('offline')))
    service.monitor(dict(service_id='receiver', observed_at=100., rtcm_bytes=1000,
                         pid=123, fix=None))
    assert 'GPS 서비스 연결 끊김' in capsys.readouterr().out


def test_multiple_route_clients_keep_the_same_fix_generation(monkeypatch):
    snapshot = dict(service_id='receiver', observed_at=100.,
                    fix=[37.5, 127., 30., 4, .1, 99.9], cog=[1., .2, .1],
                    satellites=[24, .6], rtcm_bytes=500, nmea_bytes=600, reset_count=0)
    now = [100.]
    monkeypatch.setattr(client.time, 'monotonic', lambda: now[0])
    calls = []
    monkeypatch.setattr(client, 'request', lambda: calls.append(1) or snapshot)
    first = client.PersistentGgaLink()
    assert first.latest_fix()[3:] == pytest.approx((4, .1, 99.9))
    first.stop()
    now[0] = 100.4
    second = client.PersistentGgaLink()
    assert second.latest_fix()[3:] == pytest.approx((4, .5, 99.9))
    assert second.latest_cog()[2] == pytest.approx(.5)
    assert len(calls) == 2  # stop/start never sends a receiver reset/shutdown.
    snapshot['fix'][3] = 5
    now[0] += .1
    assert second.latest_fix()[3] == 5


def test_receiver_service_loss_cannot_refresh_old_fixed(monkeypatch):
    monkeypatch.setattr(client, 'request', lambda: (_ for _ in ()).throw(OSError('offline')))
    link = client.PersistentGgaLink()
    assert link.latest_fix() is None and link.latest_cog() is None


def test_snapshot_does_not_change_measurement_or_reset_receiver():
    receiver = NS(latest_fix=lambda: (37., 127., 5., 4, .1, 10.),
                  latest_cog=lambda: None, latest_sat_info=lambda: (20, .5),
                  rtcm_rate_and_reset=lambda: 100, nmea_rate_and_reset=lambda: 50,
                  usb_reset_count=lambda: 0)
    source = service.SnapshotSource(receiver, {'device': 'fake'})
    a, b = source.snapshot(), source.snapshot()
    assert a['fix'] == b['fix']
    assert a['service_id'] == b['service_id']
    assert b['rtcm_bytes'] == 200


def test_prepare_reuses_matching_live_service_without_spawning(monkeypatch, tmp_path):
    monkeypatch.setattr(service, 'runtime_dir', lambda: tmp_path)
    existing = {'config': {'serial_port': 'fake'}, 'pid': 123}
    monkeypatch.setattr(service, 'request', lambda **kw: existing)
    monkeypatch.setattr(service.subprocess, 'Popen', lambda *a, **kw: pytest.fail('receiver restarted'))
    assert service.ensure_running(existing['config'], 'unused') == existing
    with pytest.raises(RuntimeError, match='different settings'):
        service.ensure_running({'serial_port': 'different'}, 'unused')


def test_service_uses_detached_process_and_correct_module(monkeypatch, tmp_path):
    monkeypatch.setattr(service, 'runtime_dir', lambda: tmp_path)
    config = {'serial_port': 'fake'}
    observations = iter([OSError('offline'), {'config': config}])
    def read(**kwargs):
        result = next(observations)
        if isinstance(result, Exception):
            raise result
        return result
    monkeypatch.setattr(service, 'request', read)
    calls = []
    monkeypatch.setattr(service.subprocess, 'Popen', lambda args, **kw: calls.append((args, kw)) or NS(poll=lambda: None))
    service.ensure_running(config, '/fake/relay.py')
    args, options = calls[0]
    assert args[args.index('-m')+1] == 'stack_gps.persistent_service'
    assert options['start_new_session'] and options['close_fds']
    assert options['stdin'] == service.subprocess.DEVNULL


def test_service_ipc_survives_client_exit_and_only_explicit_stop_ends_it(monkeypatch, tmp_path):
    import multiprocessing
    import threading
    import time
    class FakeLink:
        def __init__(self, **kwargs):
            self.t = time.monotonic()
            self._thread = threading.Thread(target=lambda: None)
        def start(self): self._thread.start()
        def stop(self): pass
        def latest_fix(self): return [37., 127., 5., 4, time.monotonic()-self.t, self.t]
        def latest_cog(self): return None
        def latest_sat_info(self): return [20, .5]
        def rtcm_rate_and_reset(self): return 10
        def nmea_rate_and_reset(self): return 20
        def usb_reset_count(self): return 0
    monkeypatch.setattr(service, 'runtime_dir', lambda: tmp_path)
    monkeypatch.setattr(client, 'runtime_dir', lambda: tmp_path)
    monkeypatch.setattr(service, 'GgaLink', FakeLink)
    config = dict(serial_port='fake', baud=115200, rtcm_host='', rtcm_port=2101,
                  usb_reset_after_s=0., usb_reset_cooldown_s=0., start_relay=False)
    process = multiprocessing.get_context('fork').Process(target=service.serve, args=(config, 'unused'))
    process.start()
    try:
        deadline = time.monotonic()+3.
        while not (tmp_path / 'link.sock').exists() and time.monotonic() < deadline:
            time.sleep(.02)
        first = client.PersistentGgaLink()
        generation = first.latest_fix()[5]
        first.stop()
        second = client.PersistentGgaLink()
        assert second.latest_fix()[5] == generation
        assert process.is_alive()
        assert client.request('stop')['stopping']
        process.join(timeout=3.)
        assert process.exitcode == 0
        assert not (tmp_path / 'link.sock').exists()
    finally:
        if process.is_alive():
            process.terminate()
            process.join(timeout=3.)
