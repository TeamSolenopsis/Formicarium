from os import getcwd
from pygame import image

image = image.load(f'{getcwd()}/src/Formicarium/formicarium/robot.png')
Width = image.get_width()
Height = image.get_height()
WheelBase = Width
divisor = 3
WheelDiameter = Height / divisor
WheelRadius = WheelDiameter / 2