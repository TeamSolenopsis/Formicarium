from formicarium.Interfaces import IEnvironment, ICollider, IRobot
import pygame


class Environment(IEnvironment, ICollider):
    def __init__(self, width: float, height: float) -> None:
        super().__init__()
        self.width = width
        self.height = height
        pygame.display.set_caption("env")
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.robot_group = pygame.sprite.Group()
        self.brown = pygame.color.Color(139, 69, 19,255)

    def Update(self):
        self.robot_group.update(self.screen)
        self.robot_group.draw(self.screen)
        pygame.event.get()
        pygame.display.update()
        
        self.screen.fill(self.brown)
        pygame.draw.rect(self.screen, (0, 0, 255), pygame.Rect(0, 0, 1200, 1200), width=20)

    def AddRobot(self, robot: IRobot) -> None:
        self.robot_group.add(robot)

    def CheckCollision(self, x: float, y: float) -> bool:
        #check collision with sprite
        for robot in self.robot_group:
            if robot.rect.collidepoint(x, y):
                return True
            
        print(self.screen.get_at((int(x), int(y))))
        print(" ")
        print(self.brown)
        if self.screen.get_at((int(x), int(y))) is not self.brown:
            return True
        
        return False