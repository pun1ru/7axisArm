"""深色工程软件风格。Fusion 风格 + 调色板 + 少量 QSS，Windows / Linux 一致。"""

from __future__ import annotations

from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication

BG = "#1e2127"
PANEL = "#252931"
BASE = "#1a1d22"
BORDER = "#363b45"
TEXT = "#d7dae0"
MUTED = "#8a909c"
ACCENT = "#3d8fd1"
OK = "#4caf50"
WARN = "#e0a030"
ERR = "#e05050"
PLOT_BG = "#15171b"
GRID_ALPHA = 0.18

# 7 个关节的颜色（深色背景上可区分，色盲友好优先）
JOINT_COLORS = ("#4fc3f7", "#ffb74d", "#81c784", "#f06292", "#ba68c8", "#fff176", "#90a4ae")

QSS = f"""
QWidget {{ color: {TEXT}; font-size: 9pt; }}
QMainWindow, QDialog {{ background: {BG}; }}
QDockWidget::title {{ background: {PANEL}; padding: 4px 6px; border-bottom: 1px solid {BORDER}; }}
QGroupBox {{ border: 1px solid {BORDER}; border-radius: 4px; margin-top: 10px; padding-top: 6px; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 8px; padding: 0 4px; color: {MUTED}; }}
QToolBar {{ background: {PANEL}; border-bottom: 1px solid {BORDER}; spacing: 6px; padding: 3px; }}
QStatusBar {{ background: {PANEL}; border-top: 1px solid {BORDER}; }}
QTableView, QTreeView, QPlainTextEdit, QListView {{
    background: {BASE}; alternate-background-color: #1f2228; gridline-color: {BORDER};
    border: 1px solid {BORDER}; selection-background-color: #2f4f6f; }}
QHeaderView::section {{ background: {PANEL}; color: {MUTED}; border: 0; border-right: 1px solid {BORDER};
    border-bottom: 1px solid {BORDER}; padding: 3px 6px; }}
QPushButton {{ background: #2d323b; border: 1px solid {BORDER}; border-radius: 3px; padding: 4px 10px; }}
QPushButton:hover {{ background: #353b46; }}
QPushButton:pressed, QPushButton:checked {{ background: #2f4f6f; border-color: {ACCENT}; }}
QPushButton:disabled {{ color: #5a606b; }}
QPushButton#estop {{ background: #8b1e1e; border-color: #c03030; font-weight: bold; color: white; }}
QPushButton#estop:hover {{ background: #a52626; }}
QPushButton#estop:disabled {{ background: #3a2626; border-color: #4a3030; color: #7a6060; }}
QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {{ background: {BASE}; border: 1px solid {BORDER};
    border-radius: 3px; padding: 2px 4px; min-height: 18px; }}
QComboBox:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QLineEdit:disabled,
QCheckBox:disabled, QLabel:disabled {{ color: #5a606b; }}
QComboBox:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QLineEdit:disabled {{
    background: {BG}; border-color: #2b2f37; }}
QToolButton:disabled {{ color: #5a606b; }}
QTabWidget::pane {{ border: 1px solid {BORDER}; }}
QTabBar::tab {{ background: {PANEL}; padding: 4px 12px; border: 1px solid {BORDER}; border-bottom: 0; }}
QTabBar::tab:selected {{ background: {BG}; color: white; }}
QLabel#value {{ font-family: Consolas, "DejaVu Sans Mono", monospace; }}
QLabel#title {{ color: {MUTED}; }}
QSplitter::handle {{ background: {BORDER}; }}
"""


def apply(app: QApplication) -> None:
    app.setStyle("Fusion")
    p = QPalette()
    for role, c in [(QPalette.Window, BG), (QPalette.WindowText, TEXT), (QPalette.Base, BASE),
                    (QPalette.AlternateBase, "#1f2228"), (QPalette.Text, TEXT), (QPalette.Button, "#2d323b"),
                    (QPalette.ButtonText, TEXT), (QPalette.Highlight, "#2f4f6f"),
                    (QPalette.HighlightedText, "white"), (QPalette.ToolTipBase, PANEL),
                    (QPalette.ToolTipText, TEXT), (QPalette.PlaceholderText, MUTED)]:
        p.setColor(role, QColor(c))
    p.setColor(QPalette.Disabled, QPalette.Text, QColor("#5a606b"))
    p.setColor(QPalette.Disabled, QPalette.ButtonText, QColor("#5a606b"))
    app.setPalette(p)
    app.setStyleSheet(QSS)

    import pyqtgraph as pg
    pg.setConfigOptions(background=PLOT_BG, foreground=MUTED, antialias=False)


def mono_font_family() -> str:
    return "Consolas, 'DejaVu Sans Mono', monospace"
