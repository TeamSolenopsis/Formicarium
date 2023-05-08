from os import getcwd
from pygame import image

image = image.load(f'{getcwd()}/src/Formicarium/formicarium/robot.png')
WheelBase = image.get_width()
divosor = 3
WheelDiameter = image.get_height() / divosor
WheelRadius = WheelDiameter / 2

