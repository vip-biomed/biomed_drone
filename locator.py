#!/usr/bin/env python3
import geopy
import geocoder
from geopy.geocoders import Bing
import rospy
from mavros_msgs.msg import Waypoint
from sensor_msgs.msg import NavSatFix

global home_pos
global sub_once

api = "AiD_zlH5D9GOgr8BgvpPXVbJaUCpWkFn3SjVLNfAwvVd-wt3dn0Gy99gRQ_WkhL8"

def get_address():
    address = input("What is the address? ")
    location = geocoder.bing(address, key = api)
    return(location.latlng)


def publish_address(latlng):
    pub = rospy.Publisher('/mavros/mission_goal',Waypoint, queue_size=10)
    rate = rospy.Rate(1)
    way = Waypoint()
    way.frame = 6
    way.command = 16
    way.is_current = True
    way.autocontinue = True
    
    way.param1 = 0.0
    way.param2 = 0.0
    way.param3 = 0.0
    way.param4 = 0.0
    way.x_lat = latlng[0]
    way.y_long = -latlng[1]
    way.z_alt = 20
    publish = True
    while not rospy.is_shutdown() and publish:
        #rospy.loginfo(way)
    
        pub.publish(way)
        print(way)
        publish = False
        rate.sleep()

def publish_landing(latlng):
    pub = rospy.Publisher('/mavros/mission_goal',Waypoint, queue_size=10)
    rate = rospy.Rate(1)
    way = Waypoint()
    way.frame = 6
    way.command = 21
    way.is_current = True
    way.autocontinue = True
    
    way.param1 = 0.0
    way.param2 = 0.0
    way.param3 = 0.0
    way.param4 = 0.0
    way.x_lat = latlng[0]
    way.y_long = -latlng[1]
    way.z_alt = 0
    publish = True
    while not rospy.is_shutdown() and publish:
        #rospy.loginfo(way)
        pub.publish(way)
        print(way)
        publish = False
        rate.sleep()

def sethome(msg):
    global home_pos
    lat = msg.latitude
    long = -msg.longitude
    home_pos = (lat,long)
    sub_once.unregister()

def publish_takeoff():
    global home_pos
    pub = rospy.Publisher('/mavros/mission_goal',Waypoint, queue_size=10)
    rate = rospy.Rate(1)
    way = Waypoint()
    way.frame = 6
    way.command = 22
    way.is_current = True
    way.autocontinue = True
    
    way.param1 = 0.0
    way.param2 = 0.0
    way.param3 = 0.0
    way.param4 = 0.0
    way.x_lat = home_pos[0]
    way.y_long = home_pos[1]
    way.z_alt = 0
    publish = True
    while not rospy.is_shutdown() and publish:
        #rospy.loginfo(way)
    
        pub.publish(way)
        print(way)
        publish = False
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('locator')
    #sub_once = rospy.Subscriber('/mavros/global_position/global', NavSatFix, sethome)

    latlng = get_address()
    #publish_takeoff()
    publish_address(latlng)
    #publish_landing(latlng)

    rospy.spin()