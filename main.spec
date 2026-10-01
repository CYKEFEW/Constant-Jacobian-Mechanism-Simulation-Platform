# -*- mode: python ; coding: utf-8 -*-
"""Build with: python -m PyInstaller main.spec

Qt's built-in PyInstaller hooks collect WebEngineProcess, Qt plugins,
translations and Chromium resources. Keep the assets directory layout intact
for cj_sim.gui.ROOT / 'assets' / 'scene.html' in the one-file extraction folder.
"""
from pathlib import Path

project = Path(SPECPATH)

a = Analysis(
    [str(project / 'main.py')],
    pathex=[str(project)],
    binaries=[],
    datas=[
        (str(project / 'assets' / 'scene.html'), 'assets'),
        (str(project / 'assets' / 'scene.js'), 'assets'),
        (str(project / 'assets' / 'gpu-renderer.js'), 'assets'),
        (str(project / 'assets' / 'app.ico'), 'assets'),
        (str(project / 'assets' / 'logo.png'), 'assets'),
        (str(project / 'assets' / 'logo.svg'), 'assets'),
        (str(project / 'examples'), 'examples'),
    ],
    hiddenimports=['PySide6.QtWebEngineCore', 'PySide6.QtWebEngineWidgets',
                   'PySide6.QtWebChannel'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=['PyQt5', 'PyQt6', 'PySide2'],
    noarchive=False,
)

pyz = PYZ(a.pure)

# Including binaries/data directly in EXE (without COLLECT) creates one file.
exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='Constant Jacobian机构仿真平台',
    icon=str(project / 'assets' / 'app.ico'),
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
)
