from os import getcwd
from pygame import image

image = image.load(f'{getcwd()}/src/Formicarium/formicarium/robot.png')
WheelBase = image.get_width()
WheelRadius = (image.get_height() / 3) / 2
