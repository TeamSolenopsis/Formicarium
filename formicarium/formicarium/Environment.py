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
        self.obstacle_group = []
        self.color_background = pygame.color.Color(139, 69, 19, 255)
        self.color_laser = pygame.color.Color(255, 0, 0, 255)
        self.robot_names = []

    def update(self):
        self.robot_group.update(self.screen)
        self.robot_group.draw(self.screen)
        pygame.event.get()
        pygame.display.update()
        
        self.screen.fill(self.color_background)
        pygame.draw.rect(self.screen, (0, 0, 255), pygame.Rect(0, 0, self.width, self.height), width=20)
        for obstacle in self.obstacle_group:
            pygame.draw.rect(self.screen, (0,0,255), obstacle, width=20)
        
    def add_robot(self, robot: IRobot) -> None:
        self.robot_names.append(robot.name)
        self.robot_group.add(robot)
    
    def add_obstacle(self, pos_dim) -> None:
        _obstacle = pygame.Rect(pos_dim)
        self.obstacle_group.append(_obstacle)

    def check_collision_lidar(self, x: float, y: float) -> bool:
        #check collision with sprite
        for robot in self.robot_group:
            if robot.hitbox.collidepoint(x, y):
                return True
            
        if self.screen.get_at((x,y)) != self.color_background and self.screen.get_at((x,y)) != self.color_laser:
            return True

        return False
    
    def check_collision_robot(self, robot:pygame.sprite.Sprite) -> bool:
        _robot_group = self.robot_group.copy()
        _robot_group.remove(robot)
        for _robot in _robot_group:
            if pygame.Rect.colliderect(robot.hitbox, _robot.hitbox):
                return True
            
        for _obstacle in self.obstacle_group:
            if pygame.Rect.colliderect(robot.hitbox, _obstacle):
                return True
        
        return False
    
    def get_robot_names(self) -> list:
        return self.robot_names
