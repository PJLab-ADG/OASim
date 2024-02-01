"""
Copyright (C) 2023 by Autonomous Driving Group, Shanghai AI Laboratory.
Limited. All rights reserved.
Yan Guohang <yanguohang@pjlab.org.cn>
Luo Zhaotong <luozhaotong@pjlab.org.cn>
"""
import os
import sys


def set_env(depth: int):
    # Add project root to sys.path
    current_file_path = os.path.abspath(__file__)
    project_root_path = os.path.dirname(current_file_path)
    for _ in range(depth):
        project_root_path = os.path.dirname(project_root_path)
    if project_root_path not in sys.path:
        sys.path.append(project_root_path)
        print(f"Added {project_root_path} to sys.path")
    limsim_path = project_root_path + "/limsim/"
    if limsim_path not in sys.path:
        sys.path.append(limsim_path)
        print(f"Added {limsim_path} to sys.path")


set_env(2)


import time
from limsim.trafficManager.traffic_manager import TrafficManager
from limsim.simModel.egoTracking.model import Model
from enum import Enum
from PySide6.QtWidgets import *
from PySide6.QtGui import *
from PySide6.QtCore import *
from limsim.utils.simBase import CoordTF


class chooseType(Enum):
    Start = 1
    Normal = 2
    Arrival = 3


RED = QColor(255, 107, 129)
GREEN = QColor(39, 174, 96)
ORANGE = QColor(255, 165, 0)
DEFLAUT = QColor(255, 255, 255, 30)

global LAST_START_LANE, LAST_ARRIVAL_LANE
global START_LANE, ARRIVAL_LANE
global CHOOSE_STATE

START_LANE = ""
ARRIVAL_LANE = ""
LAST_START_LANE = ""
LAST_ARRIVAL_LANE = ""
CHOOSE_STATE = chooseType(2)


class simulationWorker(QObject):
    finished = Signal()
    updateWindow = Signal()
    planningError = Signal()

    def __init__(self, model: Model) -> None:
        super().__init__()
        self.model = model
        self.planner = TrafficManager(model)

    def run(self):
        self.model.start()
        while not self.model.tpEnd and self.model.is_gui_running:
            self.model.moveStep()
            # if self.model.timeStep % 2 == 0:
            self.updateWindow.emit()
            time.sleep(0.01)
            if self.model.timeStep % 5 == 0:
                roadgraph, vehicles = self.model.exportSce()
                if self.model.tpStart and roadgraph:
                    trajectories = self.planner.plan(
                        self.model.timeStep * 0.1, roadgraph, vehicles
                    )
                    if len(trajectories) == 0:
                        self.planningError.emit()
                        # print('Sorry. Route planning failed. Please choose another route.')
                        break
                    self.model.setTrajectories(trajectories)
                else:
                    self.model.ego.exitControlMode()
            self.model.updateVeh()

        self.model.destroy()
        self.finished.emit()


class myQGraphicsPolygonItem(QGraphicsPolygonItem):
    def __init__(self, polygon):
        super().__init__(polygon)

    def paint(self, painter: QPainter, option: QStyleOptionGraphicsItem, widget):
        painter.setPen(self.pen())
        painter.setBrush(self.brush())
        # if option.state & QStyle.StateFlag.State_Selected:
        #     global CHOOSE_STATE
        #     if CHOOSE_STATE == chooseType.Normal:
        #         painter.setBrush(QColor(255, 255, 0, 50))
        painter.drawPolygon(self.polygon())

    def mousePressEvent(self, event):
        if CHOOSE_STATE == chooseType.Start:
            global START_LANE, LAST_START_LANE
            LAST_START_LANE = START_LANE
            START_LANE = self.data(0)
        elif CHOOSE_STATE == chooseType.Arrival:
            global ARRIVAL_LANE, LAST_ARRIVAL_LANE
            LAST_ARRIVAL_LANE = ARRIVAL_LANE
            ARRIVAL_LANE = self.data(0)
        self.scene().itemClicked.emit(self)


class myQGraphicsScene(QGraphicsScene):
    itemClicked = Signal(object)


class RouteEditWindow(QMainWindow):
    genNewFrame = Signal(int)

    def __init__(
        self, net_file: str, rou_file: str, ego_veh_id: str, data_base: str = None
    ):
        super().__init__()
        self.setWindowTitle("Route Edit Workspace")
        self.resize(700, 700)
        self.setupCustomMenuBar()
        self.statusBar().setStyleSheet("QStatusBar{ color:white;}")
        self.setupMainWindow()
        self.is_running = True
        self.net_file = net_file
        self.rou_file = rou_file
        self.ego_veh_id = ego_veh_id
        self.data_base = data_base
        self.start()

    def setModel(self):
        self.model = Model(
            self.ego_veh_id,
            self.net_file,
            self.rou_file,
            ctf=self.centralWidget().ctf,
            widget=self.centralWidget(),
            dataBase=self.data_base,
            SUMOGUI=0,
            simNote="example simulation, LimSim-v-0.2.0.",
            carla_cosim=False,
        )
        self.planner = TrafficManager(self.model)

    def resume(self):
        if self.model.is_gui_running:
            self.start()

    def setLanes(self, startLaneId: str, arrivalLandId: str):
        global START_LANE, ARRIVAL_LANE
        START_LANE = startLaneId
        ARRIVAL_LANE = arrivalLandId
        self.showLaneMessage()
        self.centralWidget().scene_map.itemClicked.connect(self.showLaneMessage)

    def setupMainWindow(self):
        mainwidget = MainWidget()
        self.setCentralWidget(mainwidget)

    def setupCustomMenuBar(self):
        menuBar = QWidget()
        menuBar.setStyleSheet(
            "QPushButton {background-color: rgb(72,61,179); color: white;} \
                                    QPushButton:checked { background-color: rgb(72,61,139); }"
        )
        layout = QGridLayout()
        self.startBtn = QPushButton("Choose Start Lane")
        self.startBtn.setCheckable(True)
        self.startBtn.toggled.connect(self.chooseEdgeS)
        layout.addWidget(self.startBtn, 0, 0)
        self.destBtn = QPushButton("Choose Arrival Lane")
        self.destBtn.setCheckable(True)
        self.destBtn.toggled.connect(self.chooseEdgeD)
        layout.addWidget(self.destBtn, 0, 1)
        self.okBtn = QPushButton("Confirm")
        self.okBtn.clicked.connect(self.startSimulate)
        layout.addWidget(self.okBtn, 0, 2)
        menuBar.setLayout(layout)
        self.setMenuWidget(menuBar)

    def chooseEdgeS(self, checked):
        global CHOOSE_STATE
        if checked:
            self.destBtn.setChecked(False)
            CHOOSE_STATE = chooseType.Start
        else:
            CHOOSE_STATE = chooseType.Normal

    def chooseEdgeD(self, checked):
        global CHOOSE_STATE
        if checked:
            self.startBtn.setChecked(False)
            CHOOSE_STATE = chooseType.Arrival
        else:
            CHOOSE_STATE = chooseType.Normal

    def startSimulate(self):
        self.centralWidget().isSimulation = True
        self.startBtn.setEnabled(False)
        self.destBtn.setEnabled(False)
        self.okBtn.setEnabled(False)
        global START_LANE, ARRIVAL_LANE
        startEdge = START_LANE.split("_")[0]
        startLane = START_LANE.split("_")[1]
        endEdge = ARRIVAL_LANE.split("_")[0]
        endLane = ARRIVAL_LANE.split("_")[1]
        self.model.ego.end_lane = ARRIVAL_LANE
        self.model.findNewRoute(startEdge, startLane, endEdge, endLane)
        self.centralWidget().selectableLanes.clear()
        self.timer = QTimer()
        self.timer.timeout.connect(self.model_move_step)
        self.model.start()
        self.timer.start(10)

    def model_move_step(self):
        if not self.model.tpEnd and self.model.is_gui_running:
            self.model.moveStep()
            if self.model.tpStart and self.model.timeStep > 20:
                self.genNewFrame.emit(self.model.timeStep - 20)
            if self.model.timeStep % 5 == 0:
                roadgraph, vehicles = self.model.exportSce()
                if self.model.tpStart and roadgraph:
                    trajectories = self.planner.plan(
                        self.model.timeStep * 0.1, roadgraph, vehicles
                    )
                    if len(trajectories) == 0:
                        self.showError()
                        self.model.tpEnd = 1
                        # print('Sorry. Route planning failed. Please choose another route.')
                    else:
                        self.model.setTrajectories(trajectories)
                else:
                    self.model.ego.exitControlMode()
            if not self.model.tpEnd:
                self.model.updateVeh()
        else:
            self.timer.stop()
            for i in range(10):
                self.genNewFrame.emit(self.model.timeStep - 19 + i)
            self.model.destroy()
            # sys.exit(1)
            self.resume()

    def showError(self):
        msgBox = QMessageBox()
        msgBox.setText("Sorry. Route planning failed. Please choose another route.")
        msgBox.setWindowTitle("Error")
        msgBox.setStandardButtons(QMessageBox.Ok)
        msgBox.exec()

    def start(self):
        self.is_running = True
        self.setModel()
        self.startBtn.setEnabled(True)
        self.destBtn.setEnabled(True)
        self.okBtn.setEnabled(True)
        self.centralWidget().isSimulation = False
        self.centralWidget().scene_map.clear()
        startLaneId, arrivalLandId = self.model.selectRoute()
        # self.updateWindow()
        self.setLanes(startLaneId, arrivalLandId)

    def showLaneMessage(self):
        self.statusBar().showMessage(
            "Curent Start Lane:{}, Curent Arrival Edge:{} ".format(
                START_LANE, ARRIVAL_LANE
            )
        )
        self.centralWidget().highlightLanes()
        self.update()

    def screenshot(self, img_name: str = "screen.png"):
        pixmap = QPixmap(self.size())
        self.render(pixmap)
        pixmap.save(img_name, "PNG", -1)

    def closeEvent(self, event):
        super().closeEvent(event)
        self.model.is_gui_running = False
        time.sleep(2)


class MainWidget(QGraphicsView):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.scaleFactor = 1
        self.setRenderHint(QPainter.Antialiasing, True)
        self.ctf = CoordTF(240, self.scene_map.height())
        self.selectableLanes: dict[str, myQGraphicsPolygonItem] = {}
        self.isSimulation = False
        # self.zoom_speed: float = 1.0

    def initUI(self):
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.scene_map = myQGraphicsScene()
        self.scene_map.setSceneRect(0, 0, 600, 600)
        # self.scene_map.setBackgroundBrush(Qt.white)
        self.setScene(self.scene_map)

    def paintEvent(self, event: QPaintEvent):
        super().paintEvent(event)
        if not self.isSimulation:
            painter = QPainter(self.viewport())
            painter.setPen(QPen(QColor(255, 255, 255)))
            painter.drawText(QPoint(60, 30), "Start Lane")
            painter.drawText(QPoint(60, 50), "Arrival Lane")
            painter.setBrush(GREEN)
            painter.drawRect(20, 20, 30, 10)
            painter.setBrush(RED)
            painter.drawRect(20, 40, 30, 10)

    def wheelEvent(self, event: QWheelEvent) -> None:
        if len(self.scene().items()) == 0:
            return

        curPoint = event.position()
        scenePos = self.mapToScene(QPoint(curPoint.x(), curPoint.y()))

        viewWidth = self.viewport().width()
        viewHeight = self.viewport().height()

        hScale = curPoint.x() / viewWidth
        vScale = curPoint.y() / viewHeight

        wheelDeltaValue = event.angleDelta().y()
        self.scaleFactor *= self.transform().m11()
        if (self.scaleFactor < 0.05 and wheelDeltaValue < 0) or (
            self.scaleFactor > 50 and wheelDeltaValue > 0
        ):
            return

        if wheelDeltaValue > 0:
            self.scale(1.1, 1.1)
            # self.zoom_speed *= 1.1
        else:
            self.scale(1.0 / 1.1, 1.0 / 1.1)

        viewPoint = self.transform().map(scenePos)
        self.horizontalScrollBar().setValue(int(viewPoint.x() - viewWidth * hScale))
        self.verticalScrollBar().setValue(int(viewPoint.y() - viewHeight * vScale))

    def highlightLanes(self):
        if LAST_START_LANE != "":
            lastStartLane = self.selectableLanes[LAST_START_LANE]
            lastStartLane.setBrush(DEFLAUT)
        if LAST_ARRIVAL_LANE != "":
            lastArrivalLane = self.selectableLanes[LAST_ARRIVAL_LANE]
            lastArrivalLane.setBrush(DEFLAUT)
        startLane = self.selectableLanes[START_LANE]
        arrivalLane = self.selectableLanes[ARRIVAL_LANE]
        if START_LANE != ARRIVAL_LANE:
            startLane.setBrush(GREEN)
            arrivalLane.setBrush(RED)
        else:
            startLane.setBrush(ORANGE)

    def drawPolyline(self, points: list[QPointF], color: QColor, width: int = 1):
        path = QPainterPath()
        polygon = QPolygonF(points)
        path.addPolygon(polygon)

        pen = QPen(color)
        pen.setWidth(width)
        new_item = QGraphicsPathItem(path, None)
        new_item.setPath(path)
        new_item.setPen(pen)
        self.scene_map.addItem(new_item)

    def drawPolygon(
        self,
        points: list[QPointF],
        pen_color: QColor,
        brush_color: QColor = QColor(0, 0, 0),
        pen_width: int = 1.0,
        is_selectable: bool = False,
        data: str = None,
    ):
        polygon = QPolygonF(points)
        pen = QPen(pen_color)
        if pen_width == 0:
            pen.setStyle(Qt.NoPen)
        else:
            pen.setWidth(pen_width)
        brush = QBrush(brush_color)
        if is_selectable:
            polygonitem = myQGraphicsPolygonItem(polygon)
            polygonitem.setFlag(QGraphicsItem.ItemIsSelectable)
            self.selectableLanes[data] = polygonitem
        else:
            polygonitem = QGraphicsPolygonItem(polygon)
        polygonitem.setPen(pen)
        polygonitem.setBrush(brush)
        if data:
            polygonitem.setData(0, data)
        self.scene_map.addItem(polygonitem)

    def drawEllipse(
        self,
        shape: list[QPointF],
        pen_color: QColor,
        brush_color: QColor,
        pen_width: int = 1.0,
    ):
        width = shape[1].x()
        height = shape[1].y()
        pen = QPen(pen_color)
        pen.setWidth(pen_width)
        brush = QBrush(brush_color)
        x = shape[0].x() - width / 2
        y = shape[0].y() - height / 2
        self.scene_map.addEllipse(x, y, width, height, pen, brush)

    def addText(
        self,
        x: float,
        y: float,
        text: str,
        color: QColor = QColor(0, 0, 0),
        fontsize: int = 12,
    ):
        font = QFont()
        font.setPixelSize(fontsize)
        t = self.scene_map.addText(text, font)
        t.setPos(QPointF(x, y))
        t.setDefaultTextColor(color)

    def clearAll(self):
        self.scene_map.clear()
        self.selectableLanes.clear()
