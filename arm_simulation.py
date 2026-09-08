#!/usr/bin/env python3
"""Embedded read-only Piper X model driven by ROS joint feedback."""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Dict, Sequence, Tuple

import numpy as np
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QLabel, QVBoxLayout, QWidget


MODEL_ROOT = (
    Path(get_package_share_directory("agx_arm_description"))
    / "agx_arm_urdf/piper_x/meshes"
)

# xyz and rpy are copied from piper_x_description.urdf.  Each mesh is already
# expressed in its link frame, so only the accumulated joint transforms are
# applied here.
JOINT_ORIGINS: Tuple[Tuple[Tuple[float, float, float], Tuple[float, float, float]], ...] = (
    ((0.0, 0.0, 0.123), (0.0, 0.0, 3.1415926)),
    ((0.0, 0.0, 0.0), (-1.5707963, -3.005806, -3.1415926)),
    ((0.28503, 0.0, 0.0), (0.0, 0.0, 2.8380798)),
    ((0.27364, 0.0, 0.0), (0.0, 0.0, 0.0806342)),
    ((0.07466, 0.0, 0.0), (-1.5707963, 1.5707963, 0.0)),
    ((0.0, -0.035, 0.0), (1.5707963, 0.0, 0.0)),
)


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the URDF fixed-axis RPY rotation matrix."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=float)
    ry = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=float)
    rz = np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=float)
    return rz @ ry @ rx


def _transform(
    xyz: Sequence[float], rpy: Sequence[float], joint_angle: float = 0.0
) -> np.ndarray:
    result = np.eye(4, dtype=float)
    result[:3, 3] = xyz
    result[:3, :3] = _rpy_matrix(*rpy) @ _rpy_matrix(0.0, 0.0, joint_angle)
    return result


def link_transforms(joints: Sequence[float]) -> Tuple[np.ndarray, ...]:
    """Calculate world transforms for base_link and link1 through link6."""
    if len(joints) != 6:
        raise ValueError("Piper X simulation requires six joint positions")
    transforms = [np.eye(4, dtype=float)]
    current = transforms[0]
    for (xyz, rpy), angle in zip(JOINT_ORIGINS, joints):
        current = current @ _transform(xyz, rpy, float(angle))
        transforms.append(current.copy())
    return tuple(transforms)


def _translation(xyz: Sequence[float]) -> np.ndarray:
    result = np.eye(4, dtype=float)
    result[:3, 3] = xyz
    return result


def gripper_transforms(
    link6_transform: np.ndarray, width_m: float
) -> Tuple[np.ndarray, ...]:
    """Calculate flange, base and two finger transforms from the official xacro."""
    width_m = min(0.1, max(0.0, float(width_m)))
    flange = link6_transform.copy()
    base = flange @ _transform((0.0, 0.0, 0.0045), (0.0, 0.0, 1.5707963))
    finger_offset = _translation((0.0, 0.0, width_m * 0.5))
    finger1 = (
        base
        @ _transform((0.0, 0.0, 0.138), (1.5707963, 0.0, 0.0))
        @ finger_offset
    )
    finger2 = (
        base
        @ _transform((0.0, 0.0, 0.138), (1.5707963, 0.0, -3.1415926))
        @ finger_offset
    )
    return flange, base, finger1, finger2


class PiperSimulationWidget(QWidget):
    """VTK model view with a safe text fallback for headless tests."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._joint_positions = [0.0] * 6
        self._gripper_width = 0.0
        self._actors: Dict[str, object] = {}
        self._vtk_widget = None
        self._renderer = None
        self._available = False
        self._render_dirty = True
        self._render_timer = QTimer(self)
        self._render_timer.setInterval(50)  # At most 20 VTK renders per second.
        self._render_timer.timeout.connect(self._render_pending)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        platform = os.environ.get("QT_QPA_PLATFORM", "").lower()
        if platform in {"offscreen", "minimal"}:
            fallback = QLabel("3D 模型\n离线测试模式")
            fallback.setObjectName("simulationFallback")
            fallback.setAlignment(Qt.AlignCenter)
            layout.addWidget(fallback)
            return

        try:
            self._build_vtk_view(layout)
            self._available = True
            self._render_timer.start()
        except Exception as exc:
            fallback = QLabel(f"3D 模型加载失败\n{exc}")
            fallback.setObjectName("simulationFallback")
            fallback.setWordWrap(True)
            fallback.setAlignment(Qt.AlignCenter)
            layout.addWidget(fallback)

    @property
    def available(self) -> bool:
        return self._available

    def _build_vtk_view(self, layout: QVBoxLayout) -> None:
        # Imports stay local so the rest of the operator UI remains usable if
        # VTK is absent or the display has no OpenGL support.
        from vtkmodules.qt.QVTKRenderWindowInteractor import (
            QVTKRenderWindowInteractor,
        )
        from vtkmodules.vtkCommonMath import vtkMatrix4x4
        from vtkmodules.vtkFiltersCore import vtkPolyDataNormals
        from vtkmodules.vtkFiltersSources import vtkPlaneSource
        from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera
        from vtkmodules.vtkIOGeometry import vtkSTLReader
        from vtkmodules.vtkRenderingAnnotation import vtkAxesActor
        from vtkmodules.vtkRenderingCore import (
            vtkActor,
            vtkLight,
            vtkPolyDataMapper,
            vtkRenderer,
        )
        import vtkmodules.vtkInteractionStyle  # noqa: F401
        import vtkmodules.vtkRenderingOpenGL2  # noqa: F401

        self._vtk_matrix_type = vtkMatrix4x4
        self._vtk_widget = QVTKRenderWindowInteractor(self)
        self._vtk_widget.setObjectName("simulationCanvas")
        layout.addWidget(self._vtk_widget)

        renderer = vtkRenderer()
        renderer.SetBackground(0.92, 0.93, 0.94)
        renderer.SetBackground2(0.72, 0.76, 0.79)
        renderer.GradientBackgroundOn()
        renderer.UseFXAAOn()
        renderer.UseSSAOOn()
        renderer.SetSSAORadius(0.045)
        renderer.SetSSAOBias(0.002)
        renderer.SetSSAOKernelSize(64)
        renderer.SSAOBlurOn()
        render_window = self._vtk_widget.GetRenderWindow()
        render_window.SetMultiSamples(4)
        render_window.AddRenderer(renderer)
        interactor = render_window.GetInteractor()
        self._interactor_style = vtkInteractorStyleTrackballCamera()
        interactor.SetInteractorStyle(self._interactor_style)
        self._renderer = renderer

        renderer.AutomaticLightCreationOff()
        key_light = vtkLight()
        key_light.SetLightTypeToSceneLight()
        key_light.SetPosition(1.6, -1.3, 1.9)
        key_light.SetFocalPoint(0.1, 0.0, 0.3)
        key_light.SetColor(1.0, 0.98, 0.95)
        key_light.SetIntensity(0.9)
        renderer.AddLight(key_light)

        fill_light = vtkLight()
        fill_light.SetLightTypeToSceneLight()
        fill_light.SetPosition(-1.0, 0.9, 1.2)
        fill_light.SetFocalPoint(0.1, 0.0, 0.3)
        fill_light.SetColor(0.88, 0.93, 1.0)
        fill_light.SetIntensity(0.48)
        renderer.AddLight(fill_light)

        names = (
            "base_link",
            "link1",
            "link2",
            "link3",
            "link4",
            "link5",
            "link6",
            "flange",
            "gripper_base",
            "gripper_link1",
            "gripper_link2",
        )
        colors = (
            (0.20, 0.27, 0.32),
            (0.12, 0.39, 0.61),
            (0.78, 0.81, 0.83),
            (0.12, 0.39, 0.61),
            (0.78, 0.81, 0.83),
            (0.24, 0.27, 0.29),
            (0.12, 0.39, 0.61),
            (0.78, 0.81, 0.83),
            (0.78, 0.81, 0.83),
            (0.12, 0.39, 0.61),
            (0.78, 0.81, 0.83),
        )
        for name, color in zip(names, colors):
            mesh_path = MODEL_ROOT / f"{name}.stl"
            if not mesh_path.is_file():
                raise FileNotFoundError(mesh_path)
            reader = vtkSTLReader()
            reader.SetFileName(str(mesh_path))
            normals = vtkPolyDataNormals()
            normals.SetInputConnection(reader.GetOutputPort())
            normals.SetFeatureAngle(50.0)
            normals.SplittingOn()
            normals.ConsistencyOn()
            normals.AutoOrientNormalsOn()
            mapper = vtkPolyDataMapper()
            mapper.SetInputConnection(normals.GetOutputPort())
            mapper.ScalarVisibilityOff()
            actor = vtkActor()
            actor.SetMapper(mapper)
            material = actor.GetProperty()
            material.SetColor(*color)
            material.SetInterpolationToPhong()
            material.SetAmbient(0.18)
            material.SetDiffuse(0.72)
            material.SetSpecular(0.22)
            material.SetSpecularPower(30.0)
            renderer.AddActor(actor)
            self._actors[name] = actor

        plane = vtkPlaneSource()
        plane.SetOrigin(-0.55, -0.55, -0.004)
        plane.SetPoint1(0.55, -0.55, -0.004)
        plane.SetPoint2(-0.55, 0.55, -0.004)
        plane.SetXResolution(12)
        plane.SetYResolution(12)
        plane_mapper = vtkPolyDataMapper()
        plane_mapper.SetInputConnection(plane.GetOutputPort())
        plane_actor = vtkActor()
        plane_actor.SetMapper(plane_mapper)
        plane_actor.GetProperty().SetColor(0.76, 0.78, 0.79)
        plane_actor.GetProperty().SetOpacity(0.55)
        plane_actor.GetProperty().SetAmbient(0.35)
        plane_actor.GetProperty().SetDiffuse(0.65)
        renderer.AddActor(plane_actor)

        grid_actor = vtkActor()
        grid_actor.SetMapper(plane_mapper)
        grid_actor.GetProperty().SetRepresentationToWireframe()
        grid_actor.GetProperty().SetColor(0.42, 0.45, 0.47)
        grid_actor.GetProperty().SetOpacity(0.28)
        renderer.AddActor(grid_actor)

        axes = vtkAxesActor()
        axes.SetTotalLength(0.16, 0.16, 0.16)
        axes.SetShaftTypeToCylinder()
        axes.SetCylinderRadius(0.025)
        axes.SetXAxisLabelText("")
        axes.SetYAxisLabelText("")
        axes.SetZAxisLabelText("")
        renderer.AddActor(axes)

        self.set_joint_positions(self._joint_positions)
        QTimer.singleShot(0, self._initialize_vtk)

    def _initialize_vtk(self) -> None:
        if self._vtk_widget is None:
            return
        self._vtk_widget.Initialize()
        self._render_pending()
        self.reset_camera()

    def _vtk_matrix(self, matrix: np.ndarray):
        result = self._vtk_matrix_type()
        for row in range(4):
            for column in range(4):
                result.SetElement(row, column, float(matrix[row, column]))
        return result

    def set_joint_positions(self, positions: Sequence[float]) -> None:
        if len(positions) != 6:
            return
        self._joint_positions = [float(value) for value in positions]
        self._render_dirty = True

    def set_gripper_width(self, width_m: float) -> None:
        self._gripper_width = min(0.1, max(0.0, float(width_m)))
        self._render_dirty = True

    def _render_pending(self) -> None:
        """Apply only the newest joint sample and render one coalesced frame."""
        if not self._render_dirty:
            return
        if not self._available and not self._actors:
            return
        transforms = link_transforms(self._joint_positions)
        names = ("base_link", "link1", "link2", "link3", "link4", "link5", "link6")
        for name, matrix in zip(names, transforms):
            self._actors[name].SetUserMatrix(self._vtk_matrix(matrix))
        gripper_names = ("flange", "gripper_base", "gripper_link1", "gripper_link2")
        matrices = gripper_transforms(transforms[-1], self._gripper_width)
        for name, matrix in zip(gripper_names, matrices):
            self._actors[name].SetUserMatrix(self._vtk_matrix(matrix))
        if self._vtk_widget is not None:
            self._vtk_widget.GetRenderWindow().Render()
        self._render_dirty = False

    def reset_camera(self) -> None:
        if self._renderer is None or self._vtk_widget is None:
            return
        camera = self._renderer.GetActiveCamera()
        camera.SetPosition(1.10, -1.20, 0.82)
        camera.SetFocalPoint(0.12, 0.0, 0.27)
        camera.SetViewUp(0.0, 0.0, 1.0)
        camera.ParallelProjectionOff()
        camera.SetViewAngle(28.0)
        self._renderer.ResetCameraClippingRange()
        self._vtk_widget.GetRenderWindow().Render()

    def shutdown(self) -> None:
        self._render_timer.stop()
        if self._vtk_widget is not None:
            try:
                self._vtk_widget.Finalize()
            except Exception:
                pass
