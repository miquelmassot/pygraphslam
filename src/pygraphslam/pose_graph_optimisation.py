import g2o
import numpy as np


class PoseGraphOptimisation(g2o.SparseOptimizer):
    """Pose graph optimisation class using g2o sparse optimisation library."""

    def __init__(self):
        """Initialize a new pose graph optimizer."""
        super().__init__()
        solver = g2o.BlockSolverSE2(g2o.LinearSolverCholmodSE2())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super().set_algorithm(solver)

    def optimize(self, max_iterations=50):
        """Optimize the pose graph."""
        super().initialize_optimization()
        super().optimize(max_iterations)

    def add_vertex(self, id, pose, fixed=False):
        """Add a new vertex to the pose graph."""
        v_se2 = g2o.VertexSE2()
        v_se2.set_id(id)
        v_se2.set_estimate(pose)
        v_se2.set_fixed(fixed)
        super().add_vertex(v_se2)

    def add_edge(
        self, vertices, measurement, information=np.identity(3), robust_kernel=None
    ):
        """Add a new edge to the pose graph."""
        edge = g2o.EdgeSE2()
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement(measurement)
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super().add_edge(edge)

    def get_pose(self, id):
        """Get the pose of a vertex in the pose graph."""
        return self.vertex(id).estimate()
