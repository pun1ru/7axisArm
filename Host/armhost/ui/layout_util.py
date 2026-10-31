"""布局小工具：让面板在空间不够时滚动，而不是把整个窗口的最小尺寸撑大。

Qt 里窗口最小尺寸 = 各停靠区 / 标签页内容最小尺寸叠加。面板直接放进去，窗口在小屏上就缩不动、
溢出屏幕，分隔条也没有可拖的余量。包一层 QScrollArea，它自己的最小尺寸很小，内容放不下时出滚动条。
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFrame, QScrollArea, QStyle, QWidget


def scrollable(w: QWidget) -> QScrollArea:
    """纵横都可滚动，无边框；放得下时和直接放没有区别。"""
    sa = QScrollArea()
    sa.setFrameShape(QFrame.NoFrame)
    sa.setWidgetResizable(True)
    sa.setWidget(w)
    return sa


def hscrollable(w: QWidget) -> QScrollArea:
    """只横向滚动的单行工具条：高度固定为内容高度；出现滚动条时再多留一条滚动条的位置。"""
    sa = QScrollArea()
    sa.setFrameShape(QFrame.NoFrame)
    sa.setWidgetResizable(True)
    sa.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    sa.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
    sa.setWidget(w)
    base = w.sizeHint().height()
    extent = sa.style().pixelMetric(QStyle.PM_ScrollBarExtent)
    bar = sa.horizontalScrollBar()

    def fit() -> None:
        sa.setFixedHeight(base + (extent if bar.maximum() > 0 else 0))

    bar.rangeChanged.connect(lambda *_: fit())
    fit()
    return sa
