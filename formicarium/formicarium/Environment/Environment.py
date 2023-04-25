from Interfaces import IEnvironment
import pygame

class Environment(IEnvironment):
    def __init__(self, width:float, height:float) -> None:
        super().__init__()
        self.width = width
        self.height = height

    def GetMap(self) -> pygame.Surface:
        pass

