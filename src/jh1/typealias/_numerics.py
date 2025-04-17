import numpy as np
from numpy import float64, uint8
from typing import Annotated, TypeVar, List
from numpy.typing import NDArray

_T = TypeVar("_T", bound=np.generic, covariant=False, contravariant=False)

uint8 = uint8

# 3-element float64 numpy array
Vec3 = Annotated[NDArray[float64], (3,)]

# 2-element float64 numpy array
Vec2 = Annotated[NDArray[float64], (2,)]

# 2-dimensional numpy array of generic type T
Array2D = Annotated[NDArray[_T], (..., 2)]

# 3-dimensional numpy array of generic type T
Array3D = Annotated[NDArray[_T], (..., 3)]

# 2-dimensional python list of generic type T
List2D = Annotated[List[_T], (..., 2)]

Mat3x3 = Annotated[NDArray[float64], (3, 3)]
