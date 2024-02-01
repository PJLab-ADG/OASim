"""
Copyright (C) 2023 by Autonomous Driving Group, Shanghai AI Laboratory.
Limited. All rights reserved.
Yan Guohang <yanguohang@pjlab.org.cn>
"""
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk
import sys


class MeshWidget(QVTKRenderWindowInteractor):
    def __init__(self, app):
        super().__init__()

        self.setFixedSize(400, 250)

        self.AddObserver("ExitEvent", lambda o, e, a=app: a.quit())
        ren = vtk.vtkRenderer()
        self.GetRenderWindow().AddRenderer(ren)
        ColorBackground = [0.0, 0.0, 0.0]
        FirstobjPath = r"hmi/mesh/carla_data#Street#street_level=0.0_res=0.3.ply"
        reader = vtk.vtkPLYReader()
        reader.SetFileName(FirstobjPath)
        reader.Update()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(reader.GetOutputPort())
        coneActor = vtk.vtkActor()
        coneActor.SetMapper(mapper)
        ren.SetBackground(ColorBackground)
        ren.AddActor(coneActor)
