"""Build the optional standalone C++ DWA numeric core.

The generated shared library is deliberately not committed.  This helper
supports the repository-local Zig toolchain used on Windows and ordinary
``clang++``/``g++`` installations on Linux and macOS.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from importlib.util import find_spec
from pathlib import Path


def _library_name() -> str:
    if sys.platform == "win32":
        return "dwa_core.dll"
    if sys.platform == "darwin":
        return "libdwa_core.dylib"
    return "libdwa_core.so"


def _zig_executable() -> Path | None:
    spec = find_spec("ziglang")
    if spec is None or spec.submodule_search_locations is None:
        return None
    executable = "zig.exe" if sys.platform == "win32" else "zig"
    candidate = Path(next(iter(spec.submodule_search_locations))) / executable
    return candidate if candidate.is_file() else None


def _compiler_command(source: Path, output: Path) -> list[str]:
    configured = os.environ.get("CXX")
    if configured:
        compiler = shutil.which(configured) or configured
    else:
        zig = _zig_executable()
        if zig is not None:
            return [str(zig), "c++", "-std=c++20", "-O3", "-shared", str(source), "-o", str(output)]
        compiler = shutil.which("clang++") or shutil.which("g++")
    if compiler is None:
        raise RuntimeError("C++20 compiler not found; install Zig, clang++, or g++")
    return [str(compiler), "-std=c++20", "-O3", "-shared", str(source), "-o", str(output)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="override shared-library path")
    args = parser.parse_args()

    lab_root = Path(__file__).resolve().parents[1]
    source = lab_root / "native" / "dwa_core.cpp"
    output = args.output or (
        lab_root / "src" / "hospital_path_lab" / "_native" / _library_name()
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    cache_root = lab_root / "outputs" / "zig-cache"
    environment = os.environ.copy()
    environment.setdefault("ZIG_GLOBAL_CACHE_DIR", str(cache_root / "global"))
    environment.setdefault("ZIG_LOCAL_CACHE_DIR", str(cache_root / "local"))
    command = _compiler_command(source, output)
    subprocess.run(command, cwd=lab_root, env=environment, check=True)
    print(output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
