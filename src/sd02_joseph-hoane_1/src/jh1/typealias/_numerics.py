import numpy as np
from typing import TypeVar, List, Any

from numpy._typing import NDArray

_T = TypeVar("_T", bound=np.generic, covariant=False, contravariant=False)

uint8 = np.uint8
float64 = np.float64

Vec3 = NDArray[np.float64]
Vec2 = NDArray[np.float64]
Array2D = NDArray[_T]
Array3D = NDArray[_T]
List2D = List[List[_T]]
Mat3x3 = NDArray[np.float64]
Mat4x4 = NDArray[np.float64]
