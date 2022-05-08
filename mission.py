#!/usr/bin/env python3
from mavros_msgs.msg import WaypointList, Waypoint, WaypointReached, Altitude, ExtendedState, HomePosition, ParamValue, State, _HomePosition
import rospy
from mavros_msgs.srv import CommandBool, SetMode, ParamGet, SetMode, WaypointClear, WaypointPush, CommandTOL, WaypointPull
from sensor_msgs.msg import NavSatFix
import cv2
from ffpyplayer.player import MediaPlayer
import time

class MavCommander:
    def __init__(self):
        #Variables
        self.altitude = Altitude()
        self.global_pos = NavSatFix()
        self.home = HomePosition()
        self.mission_wp = WaypointList()
        self.mission_goal = Waypoint()
        self.state = State()
        self.disarming_switch = False
        self.total_wps = -1
        self.disarmed = False
        self.first_run = True
        self.t2 = time.time()
        self.counter = 0
        self.video1_played = False

        #Services
        service_timeout = 30
        try:
            rospy.wait_for_service('mavros/mission/push',service_timeout)
            rospy.wait_for_service('mavros/cmd/arming',service_timeout)
            rospy.wait_for_service('mavros/mission/clear',service_timeout)
            rospy.wait_for_service('mavros/set_mode',service_timeout)
            rospy.wait_for_service('mavros/cmd/takeoff',service_timeout)
            rospy.wait_for_service('mavros/cmd/land',service_timeout)
            rospy.wait_for_service('mavros/mission/pull',service_timeout)
        except rospy.ROSException:
            print('Failed to Connect to Services')

        #Set Clients
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming',CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.wp_clear_srv = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
        self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
        self.to_srv = rospy.ServiceProxy('mavros/cmd/takeoff',CommandTOL)
        self.land_srv = rospy.ServiceProxy('mavros/cmd/land',CommandTOL)
        self.wp_pull_srv = rospy.ServiceProxy('mavros/mission/pull',WaypointPull)

        #Set Subscribers
        rospy.init_node('commander_node')
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude, self.altitude_callback)
        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.global_position_callback)
        self.home_pos_sub = rospy.Subscriber('mavros/home_position/home', HomePosition, self.home_position_callback)
        self.mission_wp_sub = rospy.Subscriber('mavros/mission/waypoints', WaypointList, self.mission_wp_callback)
        self.goal_sub = rospy.Subscriber('mavros/mission_goal',Waypoint, self.mission_goal_callback)
        self.state_sub = rospy.Subscriber('mavros/state',State,self.state_callback)

    #Callback Functions
    def altitude_callback(self,msg):
        self.altitude = msg

    def global_position_callback(self,msg):
        self.global_pos = msg

    def home_position_callback(self,msg):
        self.home = msg

    def mission_wp_callback(self,msg):
        self.mission_wp = msg

    def mission_goal_callback(self,msg):
        self.mission_goal = msg

    def state_callback(self,msg):
        self.state = msg    
        if msg.armed == True:
            self.disarming_switch = True
        if self.disarming_switch and not self.state.armed:
            self.disarmed = True
            self.play_video()

    #Functions
    def set_arm(self,arm): #Arm = true to arm, False to disarm
        current_arm = self.state.armed
        if arm and not current_arm:
            try:
                res = self.set_arming_srv(arm)
                if not res.success:
                    print("Failed to arm")
            except rospy.ServiceException as e:
                print("Failed to execute arming command")
    
    '''def clear_wps(self):
        try:
            res = self.wp_clear_srv()
            if res:
                print("Waypoints Cleared")
        except rospy.ServiceException as e:
            print("Failed to Clear Waypoints")

    def takeoff(self,alt):
        param1 = 0 #min pitch
        param2 = 0 #yaw
        param3 = self.global_pos.latitude
        param4 = self.global_pos.longitude
        param5 = alt
        try:
            res = self.to_srv(min_pitch = param1,yaw = param2,latitude=param3,longitude=param4,altitude=param5)
            if res.success:
                print("Taking Off!")
        except rospy.ServiceException as e:
            print("Failed to Takeoff")

    def land(self):
        param1 = 0 #min pitch
        param2 = 0 #yaw
        param3 = self.global_pos.latitude
        param4 = self.global_pos.longitude
        param5 = 0
        try:
            res = self.land_srv(min_pitch = param1,yaw = param2,latitude=param3,longitude=param4,altitude=param5)
            if res.success:
                print("Landing!")
        except rospy.ServiceException as e:
            print("Failed to Land")'''

    '''def get_goal(self):
        address = input("What is the address? ")
        location = geocoder.bing(address, key = api)
        way = Waypoint()
        way.frame = 6
        way.command = 16
        way.is_current = True
        way.autocontinue = True
        
        way.param1 = 0.0
        way.param2 = 0.0
        way.param3 = 0.0
        way.param4 = 0.0
        way.x_lat = location.latlng[0]
        way.y_long = location.latlng[1]
        way.z_alt = 20
        print(way)
        self.mission_goal = way

    def send_wps(self):
        wps = []
        self.get_goal()
        wps.append(self.mission_goal)
        try:
            res = self.wp_push_srv(start_index=0,waypoints = wps)
            if res.success:
                print("Waypoints Sent!")
        except rospy.ServiceException as e:
            print("Failed to upload waypoints")

    def count_wps(self):
        try:
            res = self.wp_pull_srv()
            self.total_wps = res.wp_received
            print(self.total_wps)
        except rospy.ServiceException as e:
            print("Failed to Pull Waypoints")'''

    def set_mode(self):
        mode = 216
        try:
            res = self.set_mode_srv(mode,'')
            if res.mode_sent:
                print("Mode Set")
        except rospy.ServiceException as e:
            print("Failed to set mode")

    def play_video(self):
        cap = cv2.VideoCapture('safe_vid.mp4')
        player = MediaPlayer('safe_vid.mp4')
        window_name = "video"
        cv2.namedWindow(window_name)
        video_played = False
        counter = 0
        # Check if camera opened successfully
        if (cap.isOpened()== False): 
            print("Error opening video  file")
    
        # Read until video is completed
        
        while(cap.isOpened() and (not self.video1_played)):
            ret, frame = cap.read()
            audio_frame, val = player.get_frame()
            if ret == True:
                cv2.imshow('Frame', frame)
    
            # Press Q on keyboard to  exit
                if (cv2.waitKey(25) & 0xFF == ord('q')):
                    break
                if val != 'eof' and audio_frame is not None:
                    img,t = audio_frame
            else: 
                break
        self.video1_played = True
        cap.release()
        cv2.destroyAllWindows()

        cap = cv2.VideoCapture('Narcan_administration.mp4')
        player = MediaPlayer('Narcan_administration.mp4')
        window_name = "video"
        cv2.namedWindow(window_name)                

        while((cap.isOpened()) and (self.video1_played) and (self.counter < 5)):
            ret, frame = cap.read()
            audio_frame, val = player.get_frame()
            counter += 1
            if ret == True:
                cv2.imshow('Frame', frame)
    
            # Press Q on keyboard to  exit
                if (cv2.waitKey(25) & 0xFF == ord('q')):
                    break
                if val != 'eof' and audio_frame is not None:
                    img,t = audio_frame
            else: 
                break
        self.counter += 1
        cap.release()
        cv2.destroyAllWindows()        

if __name__ == '__main__':
    Drone = MavCommander()
    rospy.spin()