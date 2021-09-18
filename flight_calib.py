#ввод библиотек
import rospy
import time
from clover import srv
from std_srvs.srv import Trigger
from threading import Thread
#подготовка всех необходимых функций
rospy.init_node('flight')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
telemetry = get_telemetry()

navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True) #взлёт на полтора метра вверх
rospy.sleep(5) #остановка на 5 сек
navigate(x=0, y=0, z=0.5, speed=0.2, frame_id='aruco_6') #выравнивание по аруко маркеру (номер зависит от карты)
rospy.sleep(5)
navigate(x=0, y=0, z=0.5, speed=0.2, frame_id='aruco_map') #полет в точку 0,5 по высоте относительно краты
rospy.sleep(5)
navigate(x=1, y=1, z=0.5, speed=0.2, frame_id='aruco_map') #полёт в точку 1 1 относительно карты
rospy.sleep(5)
navigate(x=-1, y=-1, z=0.5, speed=0.3, frame_id='aruco_map') #возврат на исходную
rospy.sleep(5)
land() # посадка
