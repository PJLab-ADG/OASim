from PySide6 import QtCore
from PySide6.QtUiTools import loadUiType
from PySide6.QtWidgets import *
from PySide6.QtGui import *
from PySide6 import QtWidgets, QtGui
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from preview import MeshWidget, PreviewWidget
from dataGenerator import GeneratorWidget
from routeEdit.gui import RouteEditWindow
from config.constants import traffic_file_paths


import sys
import os
import cv2
import numpy as np
from matplotlib import colormaps as cm
from typing import Literal


ui, _ = loadUiType("hmi/oasim.ui")
# run pyside6-rcc <your_resource.qrc> -o your_resource.py


def to_img(tensor, cam):
    return tensor.reshape([cam.intr.H, cam.intr.W, -1]).data.cpu().numpy()


def color_depth(
    depths: np.ndarray,
    scale=None,
    cmap="viridis",
    out: Literal["uint8,0,255", "float,0,1"] = "uint8,0,255",
):
    if scale is None:
        scale = depths.max() + 1e-10
    colors = cm.get_cmap(cmap)(depths / scale)[..., :3]
    if out == "uint8,0,255":
        return np.clip(((colors) * 255.0).astype(np.uint8), 0, 255)
    elif out == "float,0,1":
        return colors
    else:
        raise RuntimeError(f"Invalid out={out}")


class ImageWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        img = cv2.imread("hmi/datasets/image.png")
        rgb_fig = cv2.resize(img, (400, 300), interpolation=cv2.INTER_LINEAR)
        rgb_fig = rgb_fig.astype(np.uint8)
        temp_imgSrc = QtGui.QImage(
            rgb_fig,
            rgb_fig.shape[1],
            rgb_fig.shape[0],
            rgb_fig.shape[1] * 3,
            QtGui.QImage.Format_BGR888,
        )
        self.pix = QtGui.QPixmap.fromImage(temp_imgSrc)
        self.label = QtWidgets.QLabel(self)
        size = self.pix.size()
        self.label.setGeometry(0, 0, size.width(), size.height())
        self.label.setPixmap(self.pix)
        layout = QGridLayout()
        self.setLayout(layout)
        self.layout().addWidget(self.label, 0, 0)

    def getNewFrame(self, rendered, cam):
        rgb_volume = to_img(rendered["rgb_volume"], cam)
        rgb_fig = cv2.resize(rgb_volume, (400, 300), interpolation=cv2.INTER_LINEAR)
        rgb_fig = np.floor(rgb_fig * 255)
        rgb_fig = rgb_fig.astype(np.uint8)
        temp_imgSrc = QtGui.QImage(
            rgb_fig,
            rgb_fig.shape[1],
            rgb_fig.shape[0],
            rgb_fig.shape[1] * 3,
            QtGui.QImage.Format_RGB888,
        )
        self.pix = QtGui.QPixmap.fromImage(temp_imgSrc)

        self.label.setPixmap(self.pix)


class ImageWidget2(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.pix = QtGui.QPixmap("hmi/datasets/depth.png")
        self.label = QtWidgets.QLabel(self)
        size = self.pix.size()
        self.label.setGeometry(0, 0, size.width(), size.height())
        self.label.setPixmap(self.pix)
        layout = QGridLayout()
        self.setLayout(layout)
        self.layout().addWidget(self.label, 0, 0)

    def getNewFrame(self, rendered, cam):
        mask_volume = to_img(rendered["mask_volume"], cam)
        depth_volume = to_img(rendered["depth_volume"], cam)
        depth_max = 120
        depth_volume = (
            mask_volume * np.clip(depth_volume / depth_max, 0, 1)
            + (1 - mask_volume) * 1
        )
        depth_volume = color_depth(depth_volume.squeeze(-1), scale=1, cmap="turbo")

        rgb_fig = cv2.resize(depth_volume, (400, 300), interpolation=cv2.INTER_LINEAR)
        rgb_fig = np.floor(rgb_fig * 255)
        rgb_fig = rgb_fig.astype(np.uint8)
        temp_imgSrc = QtGui.QImage(
            rgb_fig,
            rgb_fig.shape[1],
            rgb_fig.shape[0],
            rgb_fig.shape[1] * 3,
            QtGui.QImage.Format_RGB888,
        )
        self.pix = QtGui.QPixmap.fromImage(temp_imgSrc)

        self.label.setPixmap(self.pix)


class ImageWidget3(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.pix = QtGui.QPixmap("hmi/datasets/normal.png")
        self.label = QtWidgets.QLabel(self)
        size = self.pix.size()
        self.label.setGeometry(0, 0, size.width(), size.height())
        self.label.setPixmap(self.pix)
        layout = QGridLayout()
        self.setLayout(layout)
        self.layout().addWidget(self.label, 0, 0)

    def getNewFrame(self, rendered, cam):
        normals_volume = to_img(rendered["normals_volume"], cam)
        normals_volume = (
            ((normals_volume / 2 + 0.5) * 255).clip(0, 255).astype(np.uint8)
        )

        rgb_fig = cv2.resize(normals_volume, (400, 300), interpolation=cv2.INTER_LINEAR)
        rgb_fig = np.floor(rgb_fig * 255)
        rgb_fig = rgb_fig.astype(np.uint8)
        temp_imgSrc = QtGui.QImage(
            rgb_fig,
            rgb_fig.shape[1],
            rgb_fig.shape[0],
            rgb_fig.shape[1] * 3,
            QtGui.QImage.Format_RGB888,
        )
        self.pix = QtGui.QPixmap.fromImage(temp_imgSrc)

        self.label.setPixmap(self.pix)


class OasimWindow(QtWidgets.QMainWindow, ui):
    def __init__(self):
        super(OasimWindow, self).__init__()
        # Remove default title bar
        flags = Qt.WindowFlags(
            Qt.FramelessWindowHint
        )  # | Qt.WindowStaysOnTopHint -> put windows on top
        self.setMaximumSize(1080, 720)  # (1080, 720)
        self.setWindowFlags(flags)
        self.setupUi(self)
        self.showNormal()
        self.offset = None

        self.pushButton_4.clicked.connect(self.close_win)
        self.pushButton_6.clicked.connect(self.minimize_win)
        self.pushButton_5.clicked.connect(self.mini_maximize)

        self.layout = QtWidgets.QVBoxLayout(self.frame_59)
        self.preview_widget = PreviewWidget()
        self.layout.addWidget(self.preview_widget)

        self.project_folder = os.path.dirname(os.path.dirname(__file__))

        net_file, rou_file = traffic_file_paths["CarlaTown10"]
        self.route_widget = RouteEditWindow(
            net_file=net_file,
            rou_file=rou_file,
            ego_veh_id="0",
            data_base="limsim/egoTrackingTest.db",
        )
        self.route_widget.genNewFrame.connect(self.getNewFrame)
        self.horizontalLayout_15.addWidget(self.route_widget)
        self.image_widget_img = ImageWidget()
        self.verticalLayout_6.addWidget(self.image_widget_img)

        self.image_widget_img2 = ImageWidget2()
        self.verticalLayout_7.addWidget(self.image_widget_img2)

        self.layout_60 = QtWidgets.QVBoxLayout(self.frame_60)
        self.render_widget = GeneratorWidget(self.project_folder)
        self.image_widget_img3 = ImageWidget3()
        self.layout_60.addWidget(self.image_widget_img3)

    def getNewFrame(self, timestep):
        render, cam = self.render_widget.getNewFrame(timestep)
        self.image_widget_img.getNewFrame(render, cam)
        self.image_widget_img2.getNewFrame(render, cam)
        self.image_widget_img3.getNewFrame(render, cam)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.offset = QtCore.QPoint(event.position().x(), event.position().y())
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.offset is not None and event.buttons() == Qt.LeftButton:
            self.move(
                self.pos()
                + QtCore.QPoint(event.scenePosition().x(), event.scenePosition().y())
                - self.offset
            )
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        self.offset = None
        super().mouseReleaseEvent(event)

    def close_win(self):
        self.close()

    def mini_maximize(self):
        if self.isMaximized():
            self.pushButton_5.setIcon(QIcon("./resources/icons/maximize.svg"))
            self.showNormal()
        else:
            self.pushButton_5.setIcon(QIcon("./resources/icons/minimize.svg"))
            self.showMaximized()

    def minimize_win(self):
        self.showMinimized()

    def keyPressEvent(self, a0):
        QApplication.instance().sendEvent(self.preview_widget, a0)


if __name__ == "__main__":
    app = QApplication()

    window = OasimWindow()
    window.show()

    sys.exit(app.exec())
