import numpy as np
from scipy.spatial.transform import Rotation

from jh1.typealias import Vec2, Vec3


class Transform3D:
    def __init__(
            self,
            position: Vec3,
            scale: Vec3,
            rotation: Rotation
    ):
        self.position: Vec3 = position
        self.scale: Vec3 = scale
        self.rotation: Rotation = rotation

    def apply(self, p: np.ndarray):
        pass

    @staticmethod
    def from_transform_2d(transform_2d: 'Transform2D'):
        return Transform3D(
            position=np.array([
                transform_2d.position[0],
                transform_2d.position[1],
                0
            ]),
            scale=np.full((3,), transform_2d.scale),
            rotation=Rotation.from_rotvec([0, 0, transform_2d.rotation])
        )


class Transform2D:
    def __init__(
            self,
            position: Vec2,
            scale: float,
            rotation_rad: float
    ):
        self.position: np.ndarray = position
        self.scale: float = scale
        self.rotation: float = rotation_rad

    def apply(self, p: np.ndarray):
        pass
