from formicarium.Interfaces import IEnvironment, ICollider, IRobot
import pygame
from pygame import sprite
from geometry_msgs.msg import Twist

class Environment(IEnvironment, ICollider):
    def __init__(self, width: float, height: float) -> None:
        super().__init__()
        self.width = width
        self.height = height
        pygame.display.set_caption("env")
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.robot_group = pygame.sprite.Group()
        self.color_background = pygame.color.Color(139, 69, 19, 255)
        self.color_laser = pygame.color.Color(255, 0, 0, 255)

    def Update(self):
        self.robot_group.update(self.screen)
        self.robot_group.draw(self.screen)
        pygame.event.get()
        pygame.display.update()
        
        self.screen.fill(self.color_background)
        pygame.draw.rect(self.screen, (0, 0, 255), pygame.Rect(0, 0, 1200, 1200), width=20)
        pygame.draw.rect(self.screen, (0, 0, 255), pygame.Rect(300, 300,300,300), width=20)

    def AddRobot(self, robot: IRobot) -> None:
        self.robot_group.add(robot)

    def check_collision_lidar(self, x: float, y: float) -> bool:
        #check collision with sprite
        for robot in self.robot_group:
            if robot.rect.collidepoint(x, y):
                return True
            
        if self.screen.get_at((x,y)) != self.color_background and self.screen.get_at((x,y)) != self.color_laser:
            return True

        return False
    
    def check_collision_robot(self, robot:IRobot) -> bool:
        _robot_group = self.robot_group.copy()
        _robot_group.remove(robot)
        if pygame.sprite.spritecollideany(robot, _robot_group or []):
            robot.stop()
    
