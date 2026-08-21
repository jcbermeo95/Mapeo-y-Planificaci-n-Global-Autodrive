"""
@file: bspline_curve.py
@author: Yang Haodong, Wu Maojia
@update: 2026.4.12
"""
from typing import List, Tuple, Dict, Any
import numpy as np
from scipy.interpolate import splprep, splev

from python_motion_planning.traj_optimizer.base_curve_generator import BaseCurveGenerator


class BSpline(BaseCurveGenerator):
    """
    Class for a true, smooth B-Spline curve where step controls 
    the physical arc-length spacing and k controls the degree.
    """
    def __init__(self, *args, k: int = 3, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.k = k

    def __str__(self) -> str:
        return "B-Spline Curve"

    def generate(self, points: List[Tuple[float, ...]]) -> Tuple[List[Tuple[float, float]], Dict[str, Any]]:
        """
        Generate a true, smooth B-Spline curve through input points.
        """
        if len(points) < 2:
            return [], {"success": False, "length": 0.0}

        points_2d = np.array([(float(p[0]), float(p[1])) for p in points])
        n = len(points_2d)

        # Degree cannot exceed n - 1
        effective_k = min(self.k, n - 1)
        if effective_k < 1:
            effective_k = 1

        x = points_2d[:, 0]
        y = points_2d[:, 1]

        try:
            # s=0.0 ensures a true B-spline passing smoothly through the points
            tck, u = splprep([x, y], k=effective_k, s=0.0)
        except Exception:
            return [], {"success": False, "length": 0.0}

        # 1. Compute exact arc length of the curve using a dense evaluation
        u_dense = np.linspace(0, 1, 1000)
        x_dense, y_dense = splev(u_dense, tck)
        dense_pts = np.vstack((x_dense, y_dense)).T

        diffs = np.diff(dense_pts, axis=0)
        segment_lengths = np.hypot(diffs[:, 0], diffs[:, 1])
        cumulative_lengths = np.hstack(([0.0], np.cumsum(segment_lengths)))
        total_length = cumulative_lengths[-1]

        if total_length < 1e-9:
            return [tuple(points_2d[0])], {"success": True, "length": 0.0}

        # 2. Step directly controls the physical distance between output waypoints
        step_size = self.step if self.step > 0 else 0.1
        target_distances = np.arange(0, total_length, step_size)
        
        if len(target_distances) == 0 or target_distances[-1] < total_length:
            target_distances = np.append(target_distances, total_length)

        # 3. Map physical distances back to parameter u and evaluate final smooth curve
        u_sampled = np.interp(target_distances, cumulative_lengths, u_dense)
        x_final, y_final = splev(u_sampled, tck)

        path = [(float(x_final[i]), float(y_final[i])) for i in range(len(x_final))]

        return path, {"success": True, "length": float(total_length)}
