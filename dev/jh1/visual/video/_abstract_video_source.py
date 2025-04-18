from abc import ABC, abstractmethod

from jh1.typealias import uint8, Array3D


class AbstractVideoSource(ABC):
    @abstractmethod
    def read_frame(self) -> Array3D[uint8]: raise NotImplementedError

    @abstractmethod
    def release(self): raise NotImplementedError
