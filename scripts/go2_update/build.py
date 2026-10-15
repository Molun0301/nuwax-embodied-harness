"""双端共用构建配方；依赖增量判断交给 Cargo 和 CMake。"""
import argparse
import os
import shutil
import signal
import subprocess
import sys
from pathlib import Path

from .common import NATIVE, WORKSPACE, digest, write_json

RECIPES = {
    "geometry": ("services/geometry/nuwax_geometry_runtime/Cargo.toml", "nuwax-geometry-runtime", "nuwax-geometry-runtime", ""),
    "navigation": (WORKSPACE + "Cargo.toml", "nuwax-nav-stack", "nuwax-nav-stack", "simple-2d,replanning-astar,mls"),
    "follow": (WORKSPACE + "Cargo.toml", "nuwax-nav-visual-servoing", "nuwax-person-follow", "link-liblcm"),
    "mapping": (WORKSPACE + "Cargo.toml", "nuwax-mapping-cli", "nuwax-mapping-cli", ""),
}


def build(root, cache, components, jobs, cross):
    root, cache = Path(root).resolve(), Path(cache).resolve()
    # The remote coordinator starts from a tools-only bundle before source sync.
    # Import build policy only in the child, after the selected source is ready.
    import scripts
    scripts.__path__ = [str(root / "scripts"), *scripts.__path__]
    from scripts.module_installer import hardware as hardware_probe
    resolver = getattr(hardware_probe, "resolve_build_jobs", None)
    jobs = resolver(cache, jobs) if resolver else jobs if jobs is not None else hardware_probe.detect_hardware(cache).build_jobs
    print(f"构建并行预算：{jobs}；Cargo 原生构建共享任务池。", flush=True)
    output = cache / "payload"
    output.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ)
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    sys.dont_write_bytecode = True
    # 工具链复用现有环境，输出独立持久化，避免与其它开发任务争用缓存。
    target = cache / "target"
    environment["CARGO_TARGET_DIR"] = str(target)
    environment["CARGO_BUILD_JOBS"] = str(jobs)
    environment["PATH"] = str(Path.home() / ".cargo/bin") + os.pathsep + environment["PATH"] if not cross else environment["PATH"]
    if cross and environment.get("CARGO_BUILD_TARGET") != "aarch64-unknown-linux-gnu":
        raise RuntimeError("必须先加载项目现有 env-arm64.sh 交叉环境")
    artifacts = {}

    def run(argv, cwd=None, env=None):
        command = list(argv)
        if command[0] == "cmake" and "-S" in command and cross:
            command.append("-DCMAKE_TOOLCHAIN_FILE=" + environment["CMAKE_TOOLCHAIN_FILE"])
        subprocess.run(command, cwd=str(cwd or root), env={**environment, **(env or {})}, check=True)

    def collect(source, relative):
        destination = output / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        destination.chmod(0o755)
        with destination.open("rb") as handle:
            header = handle.read(20)
        if header[:6] != b"\x7fELF\x02\x01" or int.from_bytes(header[18:20], "little") != 183:
            raise RuntimeError("产物不是 ARM64（64位ARM架构）：" + str(source))
        artifacts[relative] = {"sha256": digest(destination), "mode": 0o755}

    for name in components:
        if name in RECIPES:
            manifest, package, binary, features = RECIPES[name]
            command = ["cargo", "+1.88.0", "build", "--locked", "--release", "--jobs", str(jobs), "--manifest-path", str(root / manifest), "-p", package, "--bin", binary]
            if features:
                command += ["--features", features]
            run(command)
            collect(target / ("aarch64-unknown-linux-gnu/release" if cross else "release") / binary, "bin/" + binary)
        else:
            # 复用模块安装器的固定上游与补丁配方；仅增加交叉工具链传参。
            sys.path.insert(0, str(root))
            from scripts.module_installer.builds import _build_native
            prefix = cache / "native-install"
            binary = _build_native(root, prefix, cache / "native", name, jobs, run)
            base = NATIVE + name + "/cpp/result/"
            collect(binary, base + "bin/" + name + "_native")
            for library in (prefix / "lib").glob("liblivox*.so*"):
                collect(library, base + "lib/" + library.name)
    write_json(cache / "artifacts.json", artifacts)
    return artifacts


def supervise(command, log):
    """SSH输入断开即回收本次独立进程组；构建完成后继续持有协调锁。"""
    import select
    import time
    child = None
    stopped = False

    def stop_build():
        nonlocal stopped
        if child is None or stopped:
            return
        try:
            os.killpg(child.pid, signal.SIGTERM)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                try:
                    os.killpg(child.pid, 0)
                except ProcessLookupError:
                    break
                child.poll()
                time.sleep(0.1)
            else:
                os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait()
        stopped = True

    print("LOCKED", flush=True)
    if sys.stdin.readline().strip() != "start":
        return
    try:
        with open(log, "w") as stream:
            child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            print("BUILD_STARTED", flush=True)
            reported = False
            while True:
                status = child.poll()
                if status is not None and not reported:
                    print("BUILD_EXIT=" + str(status), flush=True)
                    reported = True
                readable, _, _ = select.select([sys.stdin], [], [], 0.5)
                if readable:
                    line = sys.stdin.readline()
                    if line.strip() == "cancel-build":
                        stop_build()
                        print("BUILD_STOPPED=" + str(child.returncode), flush=True)
                        continue
                    if not line or line.strip() in {"cancel", "release"}:
                        break
    finally:
        # 即使外层已退出也回收它留下的同组编译进程。
        stop_build()


def main():
    parser = argparse.ArgumentParser(description="Go2更新内部构建入口")
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--cache", type=Path, required=True)
    parser.add_argument("--components", default="")
    parser.add_argument("--jobs", type=int)
    parser.add_argument("--cross", action="store_true")
    parser.add_argument("--supervise", action="store_true")
    args = parser.parse_args()
    if args.jobs is not None and args.jobs < 1:
        parser.error("构建并行数须为正整数")
    if args.supervise:
        args.cache.mkdir(parents=True, exist_ok=True)
        command = [sys.executable, "-m", "scripts.go2_update.build", *[v for v in sys.argv[1:] if v != "--supervise"]]
        supervise(command, args.cache / "build.log")
    else:
        build(args.root, args.cache, list(filter(None, args.components.split(","))), args.jobs, args.cross)


if __name__ == "__main__":
    main()
