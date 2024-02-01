"""
Copyright (C) 2023 by Autonomous Driving Group, Shanghai AI Laboratory.
Limited. All rights reserved.
Yan Guohang <yanguohang@pjlab.org.cn>
Pi Jiahao <pijiahao@pjlab.org.cn>
"""
import os
import sys
from config.constants import MapOffset


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
from icecream import ic

from PySide6.QtWidgets import *
from PySide6.QtGui import *
from PySide6.QtCore import *

from nr3d_lib.config import ConfigDict
from nr3d_lib.profile import Profiler, profile
from nr3d_lib.render.volume_graphics import packed_alpha_to_vw, ray_alpha_to_vw
from nr3d_lib.models.spatial.aabb import AABBSpace
from nr3d_lib.models.spatial_accel import OccupancyGridEMA
from nr3d_lib.models.attributes.transform import RotationAxisAngle
from nr3d_lib.models.spatial_accel import OccupancyGridAS
from app.resources import load_scenes_and_assets
from app.resources.observers import Camera
from app.renderers import SingleVolumeRenderer

import time
import sqlite3

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

    renderer_ret = renderer.render(
        scene,
        observer=cam,
        show_progress=False,
        with_env=True,
        render_per_obj=True,
        render_per_obj_in_total=True,
        only_cr=False,
        bypass_ray_query_cfg=ConfigDict(
            {obj.class_name: {"forward_inv_s": forward_inv_s}}
        ),
    )

    return renderer_ret["rendered"]


class GeneratorWidget(QtWidgets.QWidget):
    def __init__(self, project_folder):
        super().__init__()

        self.dataBase = os.path.join(project_folder, "limsim/egoTrackingTest.db")

    def getNewFrame(self, timestep):
        conn = sqlite3.connect(self.dataBase)
        cur = conn.cursor()
        cur.execute(
            """SELECT frame, vid, x, y, yaw, speed, accel, laneID, lanePos, routeIdx FROM frameINFO 
            WHERE frame = {};""".format(
                timestep
            )
        )
        frameData = cur.fetchall()
        if len(frameData) > 0:
            frame_index = 0
            scene.frozen_at(frame_index)
            camera = scene.get_cameras()["front_camera"]
            for data in frameData:
                if data[1] == "0":
                    vehicle_x = data[2] - MapOffset.x_offset
                    vehicle_y = data[3] - MapOffset.y_offset
                    vehicle_yaw = data[4]
                    rpy = [0, 0, vehicle_yaw]
                    eular = R.from_euler("xyz", [rpy[0], rpy[1], rpy[2]], degrees=False)
                    rotation = eular.as_matrix()

                    translation = np.array(
                        [[vehicle_x], [vehicle_y], [MapOffset.vehicle_z]]
                    )
                    vehicle_transform = np.eye(4)
                    vehicle_transform[0:3, 0:3] = rotation
                    vehicle_transform[0:3, 3:4] = translation
                    T_frontcam = vehicle_transform @ MapOffset.T_car2frontcam
                    camera = scene.get_cameras()["front_camera"]
                    camera.intr.set_downscale(args.downscale)
                    camera.world_transform.tensor = torch.tensor(
                        T_frontcam, device=device
                    ).to(torch.float32)
                    camera.build_view_frustum()
            rgb_fig = render(camera)
            return rgb_fig, camera


# def neuralsim_init(self):
from nr3d_lib.config import BaseConfig

bc = BaseConfig()
bc.parser.add_argument(
    "--resume_dir",
    type=str,
    default="neuralsim/logs/streetsurf/all_1101_4cams_num_rays=40000_iter=21000.withmask_withlidar_exp1",
    help="Specifies the directory of the experiment to load/resume. You should always specify one of --config or --resume_dir.",
)
bc.parser.add_argument("--downscale", type=float, default=6.0)
bc.parser.add_argument("--rayschunk", type=int, default=60000)
bc.parser.add_argument("--rayschunk_for_bg", type=int, default=2**16)
bc.parser.add_argument(
    "--forward_inv_s",
    type=float,
    default=None,
    help="Bypasses the inv_s parameter for NeuS during rendering.",
)
bc.parser.add_argument(
    "--fast_render",
    type=bool,
    default=False,
    help="Enables experimental fast volume rendering with extreme parameters.",
)
args = bc.parse(print_config=False)

scene_bank, asset_bank, _ = load_scenes_and_assets(**args, device=device)
scene = scene_bank[0]
obj = scene.get_drawable_groups_by_class_name(scene.main_class_name)[0]
distant_obj = scene.get_drawable_groups_by_class_name("Distant")[0]
model = obj.model
if args.forward_inv_s is not None:
    model.ray_query_cfg.forward_inv_s = args.forward_inv_s
space: AABBSpace = model.space
obj_occ = model.accel.occ
renderer = SingleVolumeRenderer(args.renderer)
renderer.populate(asset_bank)
renderer.eval()
asset_bank.eval()

if args.fast_render:
    if isinstance(model.space, AABBSpace):
        resolution = 128
        num_step_per_occvoxel = 8
        march_step_size = (
            model.space.diameter.item() / resolution / num_step_per_occvoxel
        ) * obj.scale.ratio().max()
        ic(march_step_size)
        accel = OccupancyGridAS(
            model.space,
            device=device,
            resolution=128,
            occ_val_fn_cfg=ConfigDict(type="sdf", inv_s=256.0),
            occ_thre=0.8,
            init_cfg=ConfigDict(num_steps=128, num_pts=2**24, mode="from_net"),
        )
        # NOTE: `step_size` in world space.
        model.accel = accel
        # Run init from net
        model.accel.preprocess_per_train_step(
            0, query_fn=lambda x: model.forward_sdf(x, ignore_update=True)["sdf"]
        )
        forward_inv_s = 6400.0
        model.ray_query_cfg = ConfigDict(
            forward_inv_s=6400.0,
            query_mode="march_occ_multi_upsample_compressed",
            query_param=ConfigDict(
                nablas_has_grad=False,
                num_coarse=0,
                march_cfg=ConfigDict(
                    step_size=march_step_size,
                    max_steps=2 * resolution * num_step_per_occvoxel,
                ),
                num_fine=4,
                upsample_inv_s=64.0,
                upsample_inv_s_factors=[4, 16],
            ),
        )
    else:
        raise RuntimeError(f"Unsupported space type={type(model.space)}")

renderer.config.rayschunk = args.rayschunk
renderer.config.with_normal = True
for scene in scene_bank:
    for obs in scene.get_observers(False):
        obs.near = renderer.config.near
        obs.far = renderer.config.far

if (
    args.forward_inv_s is None
    or args.forward_inv_s.lower() == "none"
    or args.forward_inv_s.lower() == "null"
):
    forward_inv_s = None
