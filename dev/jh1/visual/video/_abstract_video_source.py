from abc import ABC, abstractmethod


class AbstractVideoSource(ABC):
    @abstractmethod
    def read_frame(self): raise NotImplementedError

    @abstractmethod
    def release(self): raise NotImplementedError
