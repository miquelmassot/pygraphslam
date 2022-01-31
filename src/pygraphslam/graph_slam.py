#!/usr/bin/env python

import copy

import g2o
import numpy as np
from scipy.spatial import cKDTree

from .gui import GraphSlamGui
from .icp import icp
from .pose_graph_optimisation import PoseGraphOptimisation


class GraphSlam:
    __slots__ = [
        "gui",
        "pose",
        "prev_odom",
        "prev_laser",
        "registered_lasers",
        "draw_last",
        "optimizer",
        "iteration_count",
    ]

    def __init__(self, save_gif, draw_last):
        self.optimizer = PoseGraphOptimisation()
        self.pose = np.eye(3)
        self.optimizer.add_vertex(0, g2o.SE2(g2o.Isometry2d(self.pose)), True)
        self.registered_lasers = []
        self.prev_odom = None
        self.prev_laser = None
        self.draw_last = draw_last
        self.gui = GraphSlamGui(save_gif)
        self.iteration_count = 0

    def enough_distance_travelled(self, dx):
        return np.linalg.norm(dx[0:2]) > 0.4 or abs(dx[2]) > 0.2

    def run_icp(self, A, B, init_pose, max_iterations=80, tolerance=0.0001):
        tran, distances, iter, cov = None, None, None, None
        with np.errstate(all="raise"):
            try:
                tran, distances, iter, cov = icp(
                    B, A, init_pose, max_iterations=max_iterations, tolerance=tolerance
                )
            except Exception:
                pass
        return tran, distances, iter, cov

    def scan_matching(self, dx, laser):
        # Scan Matching
        A = self.prev_laser
        B = laser
        x, y, yaw = dx[0], dx[1], dx[2]
        init_pose = np.array(
            [[np.cos(yaw), -np.sin(yaw), x], [np.sin(yaw), np.cos(yaw), y], [0, 0, 1]]
        )

        tran, _, _, cov = self.run_icp(A, B, init_pose)

        if tran is None:
            return

        self.pose = np.matmul(self.pose, tran)
        vertex_idx = len(self.registered_lasers)
        self.optimizer.add_vertex(vertex_idx, g2o.SE2(g2o.Isometry2d(self.pose)))
        rk = g2o.RobustKernelDCS()
        information = np.linalg.inv(cov)
        self.optimizer.add_edge(
            [vertex_idx - 1, vertex_idx],
            g2o.SE2(g2o.Isometry2d(tran)),
            information,
            robust_kernel=rk,
        )
        self.registered_lasers.append(B)

    @property
    def vertex_idx(self):
        return len(self.registered_lasers)

    def loop_closure(self):
        poses = [
            self.optimizer.get_pose(idx).to_vector()[0:2]
            for idx in range(self.vertex_idx - 1)
        ]
        kd = cKDTree(poses)

        x, y, _ = self.optimizer.get_pose(self.vertex_idx - 2).to_vector()
        idxs = kd.query_ball_point(np.array([x, y]), r=4.25)
        # For all but the last laser scan, find the best ICP pose
        for idx in idxs[:-1]:
            tran, distances, _, cov = self.run_icp(
                self.registered_lasers[idx], self.registered_lasers[-1], np.eye(3)
            )
            if tran is None:
                # print("ICP failed when trying to LC")
                continue
            information = np.linalg.inv(cov)
            if np.mean(distances) < 0.2:
                rk = g2o.RobustKernelDCS()
                self.optimizer.add_edge(
                    [self.vertex_idx, idx],
                    g2o.SE2(g2o.Isometry2d(tran)),
                    information,
                    robust_kernel=rk,
                )

        self.optimizer.optimize()
        self.pose = self.optimizer.get_pose(self.vertex_idx - 2).to_isometry().matrix()

    def draw(self):
        # Draw trajectory and map
        traj = []
        point_cloud = []

        for idx in range(max(0, self.vertex_idx - self.draw_last), self.vertex_idx):
            x = self.optimizer.get_pose(idx)
            r = x.to_isometry().R
            t = x.to_isometry().t
            filtered = self.registered_lasers[idx]
            filtered = filtered[np.linalg.norm(filtered, axis=1) < 80]
            point_cloud.append((r @ filtered.T + t[:, np.newaxis]).T)
            traj.append(x.to_vector()[0:2])
        point_cloud = np.vstack(point_cloud)
        traj = np.array(traj)

        xyreso = 0.01  # Map resolution (m)
        point_cloud = (point_cloud / xyreso).astype("int")
        point_cloud = np.unique(point_cloud, axis=0)
        point_cloud = point_cloud * xyreso
        self.gui.draw(traj, point_cloud)

    def iterate(self, odom, laser):
        self.iteration_count += 1
        # print("Iteration:", self.iteration_count)
        if self.prev_odom is None:
            # First iteration
            self.prev_odom = odom.copy()
            self.registered_lasers.append(laser)
            return

        # Compute odometry
        dx = odom - self.prev_odom

        if not self.enough_distance_travelled(dx):
            # print("Not enough distance travelled")
            return

        self.scan_matching(dx, laser)
        self.prev_odom = copy.deepcopy(odom)
        self.prev_laser = copy.deepcopy(laser)

        if self.vertex_idx > 10 and not self.vertex_idx % 10:
            self.loop_closure()

        self.draw()

    def run(self):
        self.gui.start()
        while self.gui.is_running:
            odom = self.get_odom()
            laser = self.get_laser()
            self.iterate(odom, laser)
