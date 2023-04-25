from Interfaces import IRobot, ILidar

class DiffRobot(IRobot):
    def __init__(self, lidar : ILidar) -> None:
        super().__init__()
        self.lidar = lidar