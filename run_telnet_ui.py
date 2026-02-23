#!/usr/bin/env python3
"""
Launcher simple para iniciar la Telnet UI con doble click.
"""

import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
VENV_DIR = REPO_ROOT / ".venv-telnet-ui"
VENV_PYTHON = VENV_DIR / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
REQUIREMENTS_FILE = REPO_ROOT / "tools/telnet_ui/requirements.txt"


def _running_in_target_venv() -> bool:
    try:
        return Path(sys.prefix).resolve() == VENV_DIR.resolve()
    except Exception:
        return False


def _create_venv() -> None:
    candidates = ["/usr/bin/python3", shutil.which("python3"), sys.executable]
    base_python = None
    for candidate in candidates:
        if candidate and Path(candidate).exists():
            base_python = candidate
            break
    if base_python is None:
        raise RuntimeError("No se encontro un interprete Python para crear el entorno virtual.")
    subprocess.check_call([base_python, "-m", "venv", str(VENV_DIR)])


def _reexec_in_venv() -> None:
    args = [str(VENV_PYTHON), str(Path(__file__).resolve())] + sys.argv[1:]
    os.execv(str(VENV_PYTHON), args)


def _ensure_requirements() -> None:
    try:
        import fastapi  # noqa: F401
        import uvicorn  # noqa: F401
        return
    except ImportError:
        pass

    print("Instalando dependencias en el entorno virtual...")
    subprocess.check_call([str(VENV_PYTHON), "-m", "pip", "install", "-r", str(REQUIREMENTS_FILE)])


def main() -> int:
    os.chdir(REPO_ROOT)

    if not _running_in_target_venv():
        try:
            if not VENV_PYTHON.exists():
                print("Creando entorno virtual en %s..." % VENV_DIR, flush=True)
                _create_venv()
            _reexec_in_venv()
        except Exception as exc:
            print("Error preparando entorno virtual: %s" % exc)
            return 1

    try:
        _ensure_requirements()
        import uvicorn
    except Exception as exc:
        print("Error cargando dependencias: %s" % exc)
        return 1

    print("Usando Python:", sys.executable, flush=True)
    print("Iniciando UI en http://localhost:8000 ...", flush=True)

    uvicorn.run(
        "tools.telnet_ui.app.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
