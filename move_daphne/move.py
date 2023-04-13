from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord, Angle


mycobot = MyCobot('/dev/ttyUSB0')
# mycobot.send_coords([160, 160, 160, 0, 0, 0], 70, 0)
ang = mycobot.get_angles()
print(ang)
mycobot.send_angle(Angle.J2.value, 10, 50)