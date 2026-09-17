"""Exercise route selection without loading ROS or starting any hardware."""
import os
import pty
import subprocess
from pathlib import Path

import pytest

HELPER = Path(__file__).with_name('v2_select_start.sh')
COMMAND = 'source "$1"; shift; v2_select_start "$@" || exit $?; printf "%s\\0" "${V2_LAUNCH_ARGS[@]}"'


def select(*args, terminal_input=None):
    command = ['bash', '-c', COMMAND, 'route-test', str(HELPER), *args]
    if terminal_input is None:
        return subprocess.run(command, input=b'', capture_output=True, timeout=5)
    master, slave = pty.openpty()
    try:
        process = subprocess.Popen(command, stdin=slave, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)
        os.write(master, terminal_input.encode())
        stdout, stderr = process.communicate(timeout=5)
        return subprocess.CompletedProcess(command, process.returncode, stdout, stderr)
    finally:
        os.close(master)
        os.close(slave)


def arguments(result):
    assert result.returncode == 0, result.stderr.decode()
    return result.stdout.decode().rstrip('\0').split('\0')


def test_interactive_selection_rejects_blank_and_invalid_then_accepts_short_number():
    result = select('rviz:=false', terminal_input='\n08\n4\n')
    assert arguments(result) == ['rviz:=false', 'start_waypoint:=04', 'end_waypoint:=07']
    assert result.stderr.decode().count('01~07 중 하나를 입력하세요.') == 2


def test_each_invocation_requires_a_new_selection():
    assert 'start_waypoint:=03' in arguments(select(terminal_input='03\n'))
    assert 'start_waypoint:=05' in arguments(select(terminal_input='05\n'))
    assert select().returncode == 2


def test_explicit_start_skips_prompt_and_preserves_argument_boundaries():
    result = select('start_waypoint:=04', 'run_log_dir:=/tmp/run with spaces', 'v_base:=1.0')
    assert arguments(result) == ['start_waypoint:=04', 'run_log_dir:=/tmp/run with spaces',
                                 'v_base:=1.0', 'end_waypoint:=07']
    assert '시작 경로 번호' not in result.stderr.decode()


@pytest.mark.parametrize('start', ['06', '07'])
def test_terminal_start_defaults_to_the_same_terminal_branch(start):
    assert arguments(select('start_waypoint:=' + start)) == [
        'start_waypoint:=' + start, 'end_waypoint:=' + start]


@pytest.mark.parametrize('args', [(), ('start_waypoint:=',), ('start_waypoint:=08',),
    ('start_waypoint:=03', 'start_waypoint:=04'),
    ('start_waypoint:=03', 'end_waypoint:=05'),
    ('start_waypoint:=06', 'end_waypoint:=07'),
    ('start_waypoint:=07', 'end_waypoint:=06'),
    ('start_waypoint:=03', 'end_waypoint:=06', 'end_waypoint:=07')])
def test_missing_invalid_or_conflicting_selection_never_produces_launch_arguments(args):
    result = select(*args)
    assert result.returncode == 2
    assert not result.stdout


@pytest.mark.parametrize('flag', ['--show-args', '--show-arguments', '--help', '--print-description'])
def test_read_only_launch_inspection_does_not_prompt(flag):
    assert arguments(select(flag)) == [flag]


def test_terminal_eof_cancels_instead_of_choosing_a_default():
    result = select(terminal_input='\x04')
    assert result.returncode == 2
    assert not result.stdout
