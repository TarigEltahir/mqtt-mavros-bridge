#!/usr/bin/env python

import sys
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import WaypointClear
from mavros_msgs.srv import WaypointPush, SetMode
from mavros_msgs.msg import WaypointList, Waypoint
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
import json
#import message_filters
#from std_msgs.msg import String
import paho.mqtt.client as mqtt



def on_connect(client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        client.subscribe("mission")

def on_message(client, userdata, msg):
    #print(msg.topic+"  "+ str(json.loads(msg.payload)))
    Load_Waypoints.waypoint_push_client(json.loads(msg.payload))
    Load_Waypoints.setAUTOMode()

client = mqtt.Client(client_id="Dronekit-2355dfgge43r32", clean_session=True)
client.on_connect = on_connect
client.on_message = on_message
broker_address = "e11c62b2-internet-facing-dd9ccee78c5c7c57.elb.ap-southeast-1.amazonaws.com"  # Broker address
port = 1883  # Broker port
client.username_pw_set(username="DroneKit", password="1234")
client.connect(broker_address, port)


class Drone_state_monitor:

    def __init__(self):

        rospy.init_node('Drone_state_monitor')
        self.DRONE_sub = rospy.Subscriber('/mavros/state', State, self.State_callback)
        self.DRONE_sub = rospy.Subscriber('/mavros/global_position/raw/gps_vel', TwistStamped, self.Velocity_callback)
        self.DRONE_sub = rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.Altitude_callback)
        self.DRONE_sub = rospy.Subscriber('/mavros/global_position/raw/fix', NavSatFix, self.GPS_callback)
        self.DRONE_status = "Initialising"

    def State_callback(self, msg):
        client.publish('Connection_Status' , msg.connected, qos = 0)
        client.publish('Armed' , msg.armed, qos = 0)
        client.publish('Flight_mode' , msg.mode, qos = 0)

        
    def Velocity_callback(self, msg):
        client.publish('Velocity' , str(msg.twist.linear), qos = 0)
 

    def Altitude_callback(self, msg):
        client.publish('Altitude' , msg.data, qos = 0)

    
    def GPS_callback(self, msg):
        client.publish('GPS_status' , msg.status.status, qos = 0)
        client.publish('GPS_latitude' , msg.latitude, qos = 0)
        client.publish('GPS_longitude' , msg.longitude, qos = 0)


class Load_Waypoints:

        def waypoint_clear_client():
            rospy.wait_for_service('mavros/mission/clear')
            try:
                response = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
                return response.call().success
            except rospy.ServiceException as e:
                print ("Service call failed: %s" % e)
                return False

        def waypoint_push_client(data):
            Load_Waypoints.waypoint_clear_client()
            wl = WaypointList()
            wp = Waypoint()
            x = -1
            for point in data["mission"]["items"]:
                wp = Waypoint()
                wp.frame = 3
                x = x + 1
                wp.command = int(str(data["mission"]["items"][x]["command"]))  # simple point
                if wp.command == 22:
                   wp.is_current = False
                else:
                   wp.is_current = True
                wp.autocontinue = True
                wp.param1 = 0  # takeoff altitude
                wp.param2 = 0
                wp.param3 = 0
                wp.param4 = 0
                wp.x_lat = float(data["mission"]["items"][x]["params"][4])
                wp.y_long = float(data["mission"]["items"][x]["params"][5])
                wp.z_alt = float(data["mission"]["items"][x]["params"][6])
                wl.waypoints.append(wp)
            try:
                response = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
                response(0, wl.waypoints)
                #print ('write mission success')
            except rospy.ServiceException as e:
                print ('write mission error')
                print ("Service call failed: %s" % e)

        def setAUTOMode():
            rospy.wait_for_service('/mavros/set_mode')
            try:
                flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                isModeChanged = flightModeService(custom_mode='AUTO') #return true or false
            except rospy.ServiceException as e:
                print ("service set_mode call failed: %s. AUTO Mode could not be set. Check that GPS is enabled" %e)

       
def main(args):
  sm = Drone_state_monitor()
  waypoints = Load_Waypoints()
  client.loop_forever()
  try:
    rospy.spin()  
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
