# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['simulation_file.py'],
    pathex=[],
    binaries=[],
    datas=[('Setup page.jpg', '.'), ('Result page.jpg', '.'), ('History page.jpg', '.'), ('ทดลองเปลี่ยนองศา (3_6_68).xlsx', '.')],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='simulation_file',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
