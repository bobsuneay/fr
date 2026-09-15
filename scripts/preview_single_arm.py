"""Render the actual offline Qt widgets, never connect to a robot or invent feedback."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from pathlib import Path
import sys
import yaml

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root/'src/fr3_control_panel'))
from PyQt5 import QtGui, QtWidgets
from fr3_control_panel.app import Panel

app = QtWidgets.QApplication([])
if os.name == 'nt':
    # Qt's Windows offscreen plugin does not discover system fonts on its own.
    font_path = Path(os.environ.get('WINDIR', 'C:/Windows'))/'Fonts/msyh.ttc'
    font_id = QtGui.QFontDatabase.addApplicationFont(str(font_path))
    families = QtGui.QFontDatabase.applicationFontFamilies(font_id)
    if not families:
        raise RuntimeError('Could not load Chinese font for offscreen preview')
    app.setFont(QtGui.QFont(families[0], 10))
cfg = yaml.safe_load((root/'src/fr3_control_panel/config/panel.yaml').read_text(encoding='utf-8'))
window = Panel(cfg)
window.show()
app.processEvents()
output = Path(sys.argv[1]).resolve()
output.parent.mkdir(parents=True, exist_ok=True)
if not window.grab().save(str(output)):
    raise RuntimeError('Cannot save UI preview')
print(output)
window.close()
