#!/usr/bin/env python3
"""Build the native navigation core into a shared library for the simulator.

The desktop simulator links against the *same* C++ sources that the ESP32-S3
firmware compiles, so path planning, waypoint tracking, the PID and the state
machine behave identically in both environments.

Usage
-----
    python scripts/build_native_core.py            # build if stale
    python scripts/build_native_core.py --force    # always rebuild
    python scripts/build_native_core.py --verbose  # show the compiler command
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
CORE_INCLUDE = REPO_ROOT / "navigation_core" / "include"
CORE_SRC = REPO_ROOT / "navigation_core" / "src"
BUILD_DIR = REPO_ROOT / "build" / "native"


def provenance_path() -> Path:
    """Path of the build record paired with the native library."""
    return BUILD_DIR / "navcore.provenance.json"


def library_name() -> str:
    system = platform.system()
    if system == "Windows":
        return "navcore.dll"
    if system == "Darwin":
        return "libnavcore.dylib"
    return "libnavcore.so"


def find_compiler() -> str:
    """Locate a C++ compiler, honouring $CXX and common Windows install paths."""
    explicit = os.environ.get("CXX")
    if explicit and shutil.which(explicit):
        return explicit

    for candidate in ("g++", "c++", "clang++"):
        found = shutil.which(candidate)
        if found:
            return found

    # Windows: MSYS2 installs are frequently present but not on PATH.
    if platform.system() == "Windows":
        for root in (r"C:\msys64\ucrt64\bin", r"C:\msys64\mingw64\bin", r"C:\msys64\clang64\bin"):
            for exe in ("g++.exe", "clang++.exe"):
                candidate = Path(root) / exe
                if candidate.is_file():
                    return str(candidate)
    raise SystemExit(
        "no C++ compiler found. Install MSYS2/MinGW-w64 (or set CXX) so that "
        "g++ is available, then re-run this script."
    )


def newest_source_mtime() -> float:
    sources = list(CORE_SRC.glob("*.cpp")) + list(CORE_INCLUDE.rglob("*.hpp"))
    sources.append(CORE_INCLUDE / "navigation" / "c_api.h")
    return max((path.stat().st_mtime for path in sources if path.is_file()), default=0.0)


def source_identity() -> str:
    """Hash the exact source/header set compiled into the ABI.

    Timestamps alone are insufficient: a restored checkout can have an older
    mtime than a DLL built from different contents.  The hash is deliberately
    recorded beside the artifact and checked by the simulator before loading.
    """
    paths = sorted(
        set(CORE_SRC.glob("*.cpp")) | set(CORE_INCLUDE.rglob("*.hpp")) |
        {CORE_INCLUDE / "navigation" / "c_api.h"},
        key=lambda path: path.as_posix(),
    )
    digest = hashlib.sha256()
    for path in paths:
        if not path.is_file():
            continue
        digest.update(path.relative_to(REPO_ROOT).as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def compiler_is_msvc_target(compiler: str) -> bool:
    """Whether clang is targeting the Microsoft ABI rather than GNU/MinGW."""
    try:
        output = subprocess.run([compiler, "--version"], capture_output=True,
                                text=True, check=False).stdout.lower()
    except OSError:
        return False
    return "windows-msvc" in output or "target: x86_64-pc-windows-msvc" in output


def artifact_is_current(target: Path) -> bool:
    """Return true only for a library built from the current source identity."""
    record = provenance_path()
    if not target.is_file() or not record.is_file():
        return False
    try:
        metadata = json.loads(record.read_text(encoding="utf-8"))
        return (metadata.get("source_sha256") == source_identity()
                and metadata.get("artifact_sha256") == hashlib.sha256(target.read_bytes()).hexdigest())
    except (OSError, json.JSONDecodeError):
        return False


def build(force: bool = False, verbose: bool = False) -> Path:
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    target = BUILD_DIR / library_name()

    if not force and artifact_is_current(target):
        if verbose:
            print(f"up to date: {target}")
        return target

    compiler = find_compiler()
    sources = sorted(str(path) for path in CORE_SRC.glob("*.cpp"))
    if not sources:
        raise SystemExit(f"no sources found under {CORE_SRC}")

    # Build to a temporary name, then replace the target.
    #
    # On Windows a DLL that is currently mapped by a running process cannot be
    # overwritten, but it *can* be replaced by renaming another file over it.  The
    # direct-to-target form therefore fails with "Permission denied" whenever a
    # stale Python process still holds the library, which is exactly the situation
    # during iterative development.  Writing to a scratch file and swapping avoids
    # that failure mode entirely.
    staging = target.with_name(target.name + ".new")
    if staging.exists():
        try:
            staging.unlink()
        except OSError:
            pass

    command = [
        compiler,
        "-std=c++17",
        "-O2",
        "-shared",
        "-Wall",
        "-Wextra",
        f"-I{CORE_INCLUDE}",
        "-o",
        str(staging),
        *sources,
    ]
    # GNU/MinGW needs PIC and benefits from a self-contained runtime.  LLVM's
    # Windows/MSVC target rejects those GNU-only switches, so keep the native
    # host build portable instead of assuming an MSYS2 compiler.
    if not compiler_is_msvc_target(compiler):
        command[3:3] = ["-fPIC"]
        command[7:7] = ["-static-libgcc", "-static-libstdc++", "-static"]
    if verbose:
        print(" ".join(command))
    started = time.perf_counter()
    # On Windows a compiler located outside PATH (typical for an MSYS2 install)
    # cannot load its own runtime DLLs.  Prepend its directory for the child
    # process so the build works regardless of the shell environment.
    child_env = dict(os.environ)
    compiler_dir = str(Path(compiler).resolve().parent)
    child_env["PATH"] = compiler_dir + os.pathsep + child_env.get("PATH", "")
    result = subprocess.run(command, capture_output=True, text=True, env=child_env)
    if result.returncode != 0:
        sys.stderr.write(result.stdout)
        sys.stderr.write(result.stderr)
        try:
            staging.unlink()
        except OSError:
            pass
        raise SystemExit(f"compilation failed with exit code {result.returncode}")
    if result.stderr.strip() and verbose:
        sys.stderr.write(result.stderr)

    # Swap the new library in.  os.replace() is atomic on the same volume and
    # succeeds even when the previous file is mapped by another process.
    os.replace(staging, target)
    metadata = {
        "artifact": target.name,
        "artifact_sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
        "built_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "compiler": compiler,
        "compiler_version": subprocess.run([compiler, "--version"], capture_output=True,
                                              text=True, check=False).stdout.splitlines()[0],
        "source_sha256": source_identity(),
    }
    provenance_path().write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    elapsed = time.perf_counter() - started
    if verbose:
        print(f"built {target} in {elapsed:.2f} s")
    return target


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true", help="rebuild even when up to date")
    parser.add_argument("--verbose", action="store_true", help="print the compiler command")
    parser.add_argument("--verify", action="store_true",
                        help="exit non-zero unless the artifact and provenance match current sources")
    args = parser.parse_args()
    target = BUILD_DIR / library_name()
    if args.verify:
        if artifact_is_current(target):
            print(target)
            return 0
        print("native artifact is missing, stale, or lacks matching provenance", file=sys.stderr)
        return 1
    target = build(force=args.force, verbose=args.verbose)
    # The path is printed on the last line: native_core.build_native_core() reads it.
    print(target)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
