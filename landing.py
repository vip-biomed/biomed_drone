#!/usr/bin/env python3
from mavros_msgs.msg import WaypointList, Waypoint, WaypointReached, Altitude, HomePosition, State
import rospy
from mavros_msgs.srv import CommandBool, SetMode, SetMode, WaypointPush,  WaypointPull
from sensor_msgs.msg import NavSatFix
import cv2
from ffpyplayer.player import MediaPlayer

class MavLander:
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
        self.counter = 0
        self.video1_played = False

        #Services
        service_timeout = 30
        try:
            rospy.wait_for_service('mavros/mission/push',service_timeout)
            rospy.wait_for_service('mavros/cmd/arming',service_timeout)
            rospy.wait_for_service('mavros/mission/clear',service_timeout)
            rospy.wait_for_service('mavros/set_mode',service_timeout)
            rospy.wait_for_service('mavros/cmd/land',service_timeout)
            rospy.wait_for_service('mavros/mission/pull',service_timeout)
        except rospy.ROSException:
            print('Failed to Connect to Services')

        #Set Clients
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming',CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
        self.wp_pull_srv = rospy.ServiceProxy('mavros/mission/pull',WaypointPull)

        #Set Subscribers
        rospy.init_node('lander_node')
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude, self.altitude_callback)
        self.mission_wp_sub = rospy.Subscriber('mavros/mission/waypoints', WaypointList, self.mission_wp_callback)
        self.goal_sub = rospy.Subscriber('mavros/mission_goal',Waypoint, self.mission_goal_callback)
        self.state_sub = rospy.Subscriber('mavros/state',State,self.state_callback)
        self.reached_sub = rospy.Subscriber('/mavros/mission/reached', WaypointReached, self.wp_reached_callback)

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


    def wp_reached_callback(self,msg):
        #print(msg.wp_seq)
        if msg.wp_seq == 1:
            self.play_warning_video()


    def count_wps(self):
        try:
            res = self.wp_pull_srv()
            self.total_wps = res.wp_received
            #print(self.total_wps)
        except rospy.ServiceException as e:
            print("Failed to Pull Waypoints")

    def play_warning_video(self):
        cap3 = cv2.VideoCapture('drone_landing_vid.mp4')
        player3 = MediaPlayer('drone_landing_vid.mp4')
        window_name3 = "video"
        cv2.namedWindow(window_name3)
        # Check if camera opened successfully
        if (cap3.isOpened()== False): 
            print("Error opening video  file")
    
        # Read until video is completed
        while(cap3.isOpened() and (not self.disarmed)):
            ret3, frame3 = cap3.read()
            audio_frame3, val3 = player3.get_frame()
            if ret3 == True:
                cv2.imshow('Frame', frame3)
    
            # Press Q on keyboard to  exit
                if (cv2.waitKey(25) & 0xFF == ord('q')) or (self.disarmed):
                    break
                if val3 != 'eof' and audio_frame3 is not None:
                    img,t = audio_frame3
            else: 
                break
        cap3.release()
        cv2.destroyAllWindows()        

if __name__ == '__main__':
    Drone = MavLander()
    rospy.spin()