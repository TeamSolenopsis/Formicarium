from formicarium.Interfaces import IEnvironment, ICollider, IRobot
import pygame


class Environment(IEnvironment, ICollider):
    def __init__(self, width: float, height: float) -> None:
        super().__init__()
        self.width = width
        self.height = height
        pygame.display.set_caption("env")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.robots = []

    def Update(self):
        for robot in self.robots:
            robot.Move()
            robot.Draw(self.map)

        pygame.display.update()
        brown = (139, 69, 19)
        self.map.fill(brown)
        pygame.draw.rect(self.map, (0, 0, 255), pygame.Rect(0, 0, 1200, 1200), width=20)

    def AddRobot(self, robot: IRobot) -> None:
        self.robots.append(robot)

    def CheckCollision(self, x: float, y: float) -> bool:
        return False