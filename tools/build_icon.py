"""Render the editable SVG into PNG and a multi-resolution Windows ICO.

Run: python tools/build_icon.py (uses the application's existing PySide6).
"""
import os
import struct
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtCore import QByteArray, QBuffer, QIODevice, Qt
from PySide6.QtGui import QGuiApplication, QImage, QPainter
from PySide6.QtSvg import QSvgRenderer


def main():
    app = QGuiApplication.instance() or QGuiApplication([])
    assets = Path(__file__).resolve().parents[1] / 'assets'
    renderer = QSvgRenderer(str(assets / 'logo.svg'))
    if not renderer.isValid():
        raise RuntimeError('Invalid logo.svg')
    sizes = (16, 24, 32, 48, 64, 128, 256)
    blobs = []
    for size in (*sizes, 512):
        image = QImage(size, size, QImage.Format.Format_ARGB32)
        image.fill(Qt.GlobalColor.transparent)
        painter = QPainter(image)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        renderer.render(painter)
        painter.end()
        if size == 512:
            if not image.save(str(assets / 'logo.png')):
                raise RuntimeError('PNG export failed')
            continue
        data = QByteArray()
        buffer = QBuffer(data)
        buffer.open(QIODevice.OpenModeFlag.WriteOnly)
        if not image.save(buffer, 'PNG'):
            raise RuntimeError('ICO frame export failed')
        blobs.append(bytes(data))
    offset = 6 + 16 * len(sizes)
    entries = []
    for size, blob in zip(sizes, blobs):
        entries.append(struct.pack('<BBBBHHII', size % 256, size % 256, 0, 0,
                                   1, 32, len(blob), offset))
        offset += len(blob)
    (assets / 'app.ico').write_bytes(
        struct.pack('<HHH', 0, 1, len(sizes)) + b''.join(entries) + b''.join(blobs))
    print('Generated assets/logo.png and assets/app.ico (16–256 px)')


if __name__ == '__main__':
    main()
