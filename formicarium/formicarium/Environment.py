from formicarium.Interfaces import IEnvironment
import pygame


class Environment(IEnvironment):
    def __init__(self, width: float, height: float) -> None:
        super().__init__()
        self.width = width
        self.height = height
        pygame.display.set_caption("env")
        self.map = pygame.display.set_mode((self.width, self.height))

    def GetMap(self) -> pygame.Surface:
        return self.map
