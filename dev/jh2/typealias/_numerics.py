import numpy as np
from typing import Annotated, TypeVar
from numpy.typing import NDArray

_T = TypeVar("_T", bound=np.generic)

# 3-element float64 numpy array
Vec3 = Annotated[NDArray[np.float64], (3,)]

# 2-element float64 numpy array
Vec2 = Annotated[NDArray[np.float64], (2,)]

# 2-dimensional numpy array of generic type T
Array2D = Annotated[NDArray[_T], (..., 2)]
