import abc
from pygame import Surface

class IEnvironment(abc.ABC):
    @abc.abstractmethod
    def GetMap(self) -> Surface:
        pass