"""
Copyright (C) 2023 by Autonomous Driving Group, Shanghai AI Laboratory.
Limited. All rights reserved.
Yan Guohang <yanguohang@pjlab.org.cn>
"""
import os
import sys


def set_env(depth: int):
    # Add project root to sys.path
    current_file_path = os.path.abspath(__file__)
    project_root_path = os.path.dirname(current_file_path)
    print(project_root_path)
    for _ in range(depth):
        project_root_path = os.path.dirname(project_root_path)
    project_root_path = project_root_path + "/neuralsim"
    if project_root_path not in sys.path:
        sys.path.append(project_root_path)
        print(f"Added {project_root_path} to sys.path")


set_env(2)

import cv2
from PySide6 import QtWidgets, QtGui
import PySide6.QtGui
from PySide6.QtGui import QKeyEvent, Qt
import numpy as np
import math
import scipy.linalg as linalg
from scipy.spatial.transform import Rotation as R

import json
from dash import Dash, dcc, html, Input, Output, callback
import dash_vtk
from skimage import data
import plotly.express as px

import torch
import time

from nr3d_lib.config import ConfigDict
from nr3d_lib.profile import Profiler, profile
from nr3d_lib.render.volume_graphics import packed_alpha_to_vw, ray_alpha_to_vw
from nr3d_lib.models.spatial.aabb import AABBSpace
from nr3d_lib.models.spatial_accel import OccupancyGridEMA
from nr3d_lib.models.attributes.transform import RotationAxisAngle

from app.resources import load_scenes_and_assets
from app.resources.observers import Camera
from app.renderers import SingleVolumeRenderer

device = torch.device("cuda", 0)
torch.set_grad_enabled(False)

scene = None
cam = None
obj = None
renderer = None
renderer_ret = None
rgb_fig = None
sp_rgb_fig = None
fg_query_cfg = None
x = None
y = None


def img_to_fig(img: torch.Tensor):
    return img.cpu().numpy()


@profile
def render(cam, sphere_trace=False):
    global renderer_ret
    if sphere_trace:
        bypass_ray_query_cfg = ConfigDict(
            {
                obj.class_name: ConfigDict(
                    {
                        "query_mode": "sphere_trace",
                        "query_param": ConfigDict(debug=True),
                    }
                )
            }
        )
        renderer_ret = renderer.render(
            scene,
            observer=cam,
            render_per_obj=True,
            rayschunk=0,
            bypass_ray_query_cfg=bypass_ray_query_cfg,
        )
    else:
        renderer_ret = renderer.render(
            scene, observer=cam, render_per_obj=True, rayschunk=args.rayschunk
        )
    return renderer_ret["rendered"]["rgb_volume"].reshape(cam.intr.H, cam.intr.W, 3)


modify_matrix_list_ = {}
modify_scale_degree_ = 3
modify_scale_trans_ = 3

current_matrix = np.eye(4)


def rotate_mat(axis, radian):
    rot_matrix = linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))
    return rot_matrix


def modify_matrix_init():
    for i in range(12):
        transform_flag = [0, 0, 0, 0, 0, 0]
        transform_flag[i // 2] = 1 if i % 2 == 0 else -1
        # print(transform_flag[i // 2])
        rand_axis_x = [1, 0, 0]
        rand_axis_y = [0, 1, 0]
        rand_axis_z = [0, 0, 1]

        tmp_tansform = np.eye(4)

        rot_tmp_x = rotate_mat(
            rand_axis_x, transform_flag[0] * modify_scale_degree_ / 180 * math.pi
        )
        rot_tmp_y = rotate_mat(
            rand_axis_y, transform_flag[1] * modify_scale_degree_ / 180 * math.pi
        )
        rot_tmp_z = rotate_mat(
            rand_axis_z, transform_flag[2] * modify_scale_degree_ / 180 * math.pi
        )
        rot_tmp = np.dot(np.dot(rot_tmp_x, rot_tmp_y), rot_tmp_z)

        tmp_tansform[:3, :3] = rot_tmp
        tmp_tansform[0, 3] = transform_flag[3] * modify_scale_trans_
        tmp_tansform[1, 3] = transform_flag[4] * modify_scale_trans_
        tmp_tansform[2, 3] = transform_flag[5] * modify_scale_trans_
        modify_matrix_list_[i] = tmp_tansform


def ManualControl(key_input):
    table = ["q", "a", "w", "s", "e", "d", "r", "f", "t", "g", "y", "h"]
    global current_matrix
    for i in range(12):
        if key_input == table[i]:
            current_matrix = np.dot(current_matrix, modify_matrix_list_[i])
            # print(current_matrix)
            return current_matrix


class PreviewWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        frame_index = 0
        scene.frozen_at(frame_index)
        camera = scene.get_cameras()["front_camera"]
        camera.intr.set_downscale(args.downscale)
        rays_o, rays_d = camera.get_all_rays()
        rays_o = rays_o.reshape(camera.intr.H, camera.intr.W, -1)
        rays_d = rays_d.reshape(camera.intr.H, camera.intr.W, -1)
        rgb_fig = img_to_fig(render(camera))
        rgb_fig = cv2.resize(rgb_fig, (400, 300), interpolation=cv2.INTER_LINEAR)
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
        size = self.pix.size()

        self.label1 = QtWidgets.QLabel(self)
        self.label1.setGeometry(0, 0, size.width(), size.height())
        self.label1.setPixmap(self.pix)
        modify_matrix_init()

    def keyPressEvent(self, event: QKeyEvent):
        current_matrix = ManualControl(event.text())
        frame_index = 0
        scene.frozen_at(frame_index)
        camera = scene.get_cameras()["front_camera"]
        camera.intr.set_downscale(args.downscale)

        rot_tmp = current_matrix[:3, :3]
        tran_tmp = current_matrix[:3, 3]
        # print(tran_tmp)
        camera.world_transform.tensor[..., :3, :3] = camera.world_transform.tensor[
            ..., :3, :3
        ] @ torch.tensor(rot_tmp, device=device).to(torch.float32)
        camera.world_transform.tensor[..., :3, 3] = camera.world_transform.tensor[
            ..., :3, 3
        ] + torch.tensor(tran_tmp, dtype=torch.float, device=device)

        rgb_fig = img_to_fig(render(camera))
        rgb_fig = cv2.resize(rgb_fig, (400, 300), interpolation=cv2.INTER_LINEAR)
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
        self.label1.setPixmap(self.pix)


from nr3d_lib.config import BaseConfig

bc = BaseConfig()
bc.parser.add_argument("--downscale", type=float, default=4.0)
bc.parser.add_argument("--rayschunk", type=int, default=60000)
bc.parser.add_argument("--rayschunk_for_bg", type=int, default=2**16)
args = bc.parse(print_config=False)
scene_bank, asset_bank, _ = load_scenes_and_assets(**args, device=device)
scene = scene_bank[0]
obj = scene.get_drawable_groups_by_class_name(scene.main_class_name)[0]
obj_model = obj.model
obj_space: AABBSpace = obj_model.space
obj_occ = obj_model.accel.occ
renderer = SingleVolumeRenderer(args.renderer)
renderer.populate(asset_bank)
renderer.eval()

renderer.config.rayschunk = args.rayschunk
renderer.config.with_normal = False
for scene in scene_bank:
    for obs in scene.get_observers(False):
        obs.near = renderer.config.near
        obs.far = renderer.config.far

fg_query_cfg = obj.model.ray_query_cfg
fg_query_cfg["forward_inv_s"] = obj.model.forward_inv_s().item()
