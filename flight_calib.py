import rospy
import time
from clover import srv
from std_srvs.srv import Trigger
from threading import Thread


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
rospy.sleep(5)
navigate(x=0, y=0, z=0.5, speed=0.2, frame_id='aruco_6')
rospy.sleep(5)
navigate(x=0, y=0, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(5)
navigate(x=1, y=1, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(5)
navigate(x=-1, y=-1, z=0.5, speed=0.3, frame_id='aruco_map')
rospy.sleep(5)
land()
