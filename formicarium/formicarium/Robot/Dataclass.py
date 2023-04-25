from dataclasses import dataclass
from rclpy.publisher import Publisher

@dataclass
class Position:
    x: float
    y: float

@dataclass
class RobotConfig:
    WheelRadius: float
    WheelBase:float
    StartPose: Position
    OdomPublisher: Publisher
    PosePublisher: Publisher

@dataclass
class LidarConfig:
    Range: float
    Position: Position
    ScanPublisher: Publisher