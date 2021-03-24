import rospy
import time
from clover import srv
from std_srvs.srv import Trigger
from threading import Thread

Land = False


def landing():
  res = input()
  while res != "l":
    res = input()
  Land = True

rospy.init_node('flight')

FocusAlt = 1.6
Coords = [
[0.5, 0],
[0.5, 0],
[0.5, 0],
[0.5, 0]
]
th = Thread(target=landing)
th.start()
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
telemetry = get_telemetry()
navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(15)
for i in range(len(Coords)):
  elem = Coords[i]
  print(i + 1, "point")
  navigate(x=elem[0], y=elem[1], z=0, speed=0.09, frame_id='body')
  rospy.sleep(10)
  if Land:
    break
    rospy.sleep(5)

land()



