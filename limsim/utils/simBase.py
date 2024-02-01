import dearpygui.dearpygui as dpg
from PySide6.QtGui import QColor
from PySide6.QtCore import QPointF

class graphicsItem:
    def __init__(self, 
                 type: str, 
                 shape: list[QPointF], 
                 pen_color: QColor, 
                 brush_color: QColor, 
                 pen_width: int = 1, 
                 is_selectable: bool = False, 
                 data: str = ""):
        self.type = type
        self.shape = shape
        self.pen_color = pen_color
        self.brush_color = brush_color
        self.pen_width = pen_width
        self.is_selectable = is_selectable
        self.data = data

class CoordTF:
    # Ego is always in the center of the window
    def __init__(self, realSize: float, height:float) -> None:
        self.realSize: float = realSize
        self.drawCenter: float = self.realSize / 2
        self.qtDrawSize: float = height
        self.offset = (0, 0)

    @property
    def zoomScale(self) -> float:
        return self.qtDrawSize / self.realSize

    def qtCoord(
            self, x: float, y: float, ex: float, ey: float) -> tuple[QPointF]:
        relx, rely = x - ex, y - ey
        return QPointF(
            self.zoomScale * (self.drawCenter + relx + self.offset[0]),
            self.zoomScale * (self.drawCenter - rely + self.offset[1])
        )


def deduceEdge(laneID: str) -> str:
    slist = laneID.split('_')
    del (slist[-1])
    return '_'.join(slist)


class vehType:
    def __init__(self, id: str) -> None:
        self.id = id
        self.maxAccel = None
        self.maxDecel = None
        self.maxSpeed = None
        self.length = None
        self.width = None
        self.vclass = None

    def __str__(self) -> str:
        return 'ID: {},vClass: {}, maxAccel: {}, maxDecel: {:.2f}, maxSpeed: {:.2f}, length: {:.2f}, width: {:.2f}'.format(
            self.id, self.vclass, self.maxAccel, self.maxDecel, self.maxSpeed, self.length, self.width
        )
