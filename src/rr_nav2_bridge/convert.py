"""Conversions between RR namedarrays/structs and ROS messages, plus small math helpers."""
from __future__ import annotations

import math

import numpy as np


def yaw_to_quat(yaw: float):
    """(x, y, z, w) quaternion for a rotation of yaw about +Z."""
    h = 0.5 * yaw
    return 0.0, 0.0, math.sin(h), math.cos(h)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(a: float) -> float:
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def pose2d_tuple(p) -> tuple:
    """RR Pose2D (numpy structured scalar or shape-(1,) array) -> (x, y, yaw)."""
    p = np.asarray(p).reshape(-1)[0]
    return float(p["position"]["x"]), float(p["position"]["y"]), float(p["orientation"])


def finite(*vals) -> bool:
    return all(math.isfinite(v) for v in vals)


class Types:
    """Factory for RR values; bound to a RobotRaconteurNode after types are registered."""

    def __init__(self, node):
        self.node = node
        self.pose2d_dt = node.GetNamedArrayDType("com.robotraconteur.geometry.Pose2D")
        self.point2d_dt = node.GetNamedArrayDType("com.robotraconteur.geometry.Point2D")
        self.spatial_vel_dt = node.GetNamedArrayDType("com.robotraconteur.geometry.SpatialVelocity")
        self.pointf_dt = node.GetNamedArrayDType("com.robotraconteur.geometryf.Point")
        ac = node.GetConstants("com.robotraconteur.action")["ActionStatusCode"]
        self.status = ac  # dict name -> int
        self.consts = node.GetConstants("experimental.nav2_bridge")

    def struct(self, name: str):
        if "." not in name:
            name = "experimental.nav2_bridge." + name
        return self.node.NewStructure(name)

    def pose2d(self, x=0.0, y=0.0, yaw=0.0):
        a = np.zeros((1,), dtype=self.pose2d_dt)
        a[0]["position"]["x"] = x
        a[0]["position"]["y"] = y
        a[0]["orientation"] = yaw
        return a

    def pose2d_array(self, poses):
        a = np.zeros((len(poses),), dtype=self.pose2d_dt)
        for i, (x, y, yaw) in enumerate(poses):
            a[i]["position"]["x"] = x
            a[i]["position"]["y"] = y
            a[i]["orientation"] = yaw
        return a

    def velocity(self, vx=0.0, vy=0.0, wz=0.0):
        a = np.zeros((1,), dtype=self.spatial_vel_dt)
        a[0]["linear"]["x"] = vx
        a[0]["linear"]["y"] = vy
        a[0]["angular"]["z"] = wz
        return a

    def point2d_array(self, pts):
        a = np.zeros((len(pts),), dtype=self.point2d_dt)
        for i, (x, y) in enumerate(pts):
            a[i]["x"] = x
            a[i]["y"] = y
        return a

    def path2d(self, poses, stamp: float):
        s = self.struct("Path2D")
        s.poses = self.pose2d_array(poses)
        s.stamp = float(stamp)
        s.length = path_length(poses)
        return s


def path_length(poses) -> float:
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poses, poses[1:])))
