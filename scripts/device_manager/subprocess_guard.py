"""安装工作进程退出时收回其命令进程组，不留下继续写候选环境的孤儿进程。"""
import ctypes
import os
import signal
import subprocess
import sys


def terminate_group(_signal, _frame):
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    try:
        os.killpg(os.getpgrp(), signal.SIGTERM)
    finally:
        # 组内任何忽略终止信号的安装后代也不能在事务恢复后继续写入。
        os.killpg(os.getpgrp(), signal.SIGKILL)


def main():
    parent = int(sys.argv[1])
    signal.signal(signal.SIGTERM, terminate_group)
    ctypes.CDLL(None).prctl(1, signal.SIGTERM)
    if os.getppid() != parent:
        terminate_group(signal.SIGTERM, None)
    command = subprocess.Popen(sys.argv[2:], stdin=sys.stdin.buffer, stdout=sys.stdout.buffer, stderr=sys.stderr.buffer)
    return command.wait()


if __name__ == "__main__":
    raise SystemExit(main())
