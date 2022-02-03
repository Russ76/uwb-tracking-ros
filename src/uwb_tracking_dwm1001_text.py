#!/usr/bin/env python3
""" 
    This src is adapted from the following repo, which is under the MIT license:
    https://github.com/TIERS/ros-dwm1001-uwb-localization
    For more info on the documentation of DWM1001, go to the following links from Decawave: 
    1. https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
    2. https://www.decawave.com/product-documentation/    
    Prog modified to work with either active tag or passive listener
    This version uses text input to simulate UWB network. Use it to refine Rviz integration.
"""
import rospy, time, serial, os, csv
from dwm1001_apiCommands import DWM1001_API_COMMANDS
from geometry_msgs.msg import PoseStamped
from KalmanFilter import KalmanFilter as kf
import numpy as np
from Helpers_KF import initConstVelocityKF 
from uwb_tracking_ros.msg import CustomTag
from uwb_tracking_ros.msg import MultiTags

class dwm1001_localizer:

    def __init__(self) :
        """
        Initialize the node, open serial port
        """        
        # Init node
        rospy.init_node('DWM1001_Listener_Mode', anonymous=False)

        # allow serial port to be detected by user
        # NOTE: USB is assumed to be connected to ttyACM0. If not, need to modified it.
        # os.popen("sudo chmod 777 /dev/ttyACM0", "w")  
        
        # Set a ROS rate
        self.rate = rospy.Rate(1)
        
        # Empty dictionary to store topics being published
        self.topics = {}
        self.topics_kf = {}
        # Empty list for each tags of Kalman filter 
        self.kalman_list = [] 

        self.multipleTags = MultiTags()
        self.pub_tags = rospy.Publisher("/dwm1001/multiTags", MultiTags, queue_size=100) 
         
        # Serial port settings
        # self.dwm_port = rospy.get_param('~port')
        # self.verbose = rospy.get_param('~verbose', True)
        # self.use_network = False
        # self.serialPortDWM1001 = serial.Serial(port = self.dwm_port, baudrate = 115200, parity = serial.PARITY_ODD, stopbits = serial.STOPBITS_TWO, bytesize = serial.SEVENBITS)

    
    def main(self) :
        """
        Initialize port and dwm1001 api
        :param:
        :returns: none
        """
        # close the serial port in case the previous run didn't closed it properly
        # self.serialPortDWM1001.close()
        # sleep for one sec
        time.sleep(2)
        # open serial port
        # self.serialPortDWM1001.open()
        # opening a text file
        file1 = open("/home/russ/save_passive_input.txt", "r")
        time.sleep(1)

        # check if the serial port is opened
        # if(self.serialPortDWM1001.isOpen()):
            # rospy.loginfo("Port opened: "+ str(self.serialPortDWM1001.name) )
            # start sending commands to the board so we can initialize the board
            # self.initializeDWM1001API()
            # give some time to DWM1001 to wake up
            # time.sleep(2)
            # send command lec, so we can get positions is CSV format
            # self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)
            # self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            # rospy.loginfo("Reading DWM1001 coordinates and process them!")
        # else:
            # rospy.loginfo("Can't open port: "+ str(self.serialPortDWM1001.name))

        while not rospy.is_shutdown():
            # just read everything from serial port
            # serialReadLine = self.serialPortDWM1001.read_until()
            
            for line in file1:
                serialReadLine = line

                try:
                    # Publish the Raw Pose Data directly from the USB
                    rospy.loginfo(serialReadLine)
                    time.sleep(.1)
                    # Check what kind of device we're plugged into, publish data
                    self.publishTagPositions(serialReadLine)    

                    ############### Kalman Filter ###############
                    # Use Kalman filter to process the data and publish it 
                    ps = PoseStamped()
                    t_pose_x = ps.pose.position.x
                    t_pose_y = ps.pose.position.y
                    t_pose_z = ps.pose.position.z
                    tag_macID = "1AA1"
                    tag_id = 0

                    # To use this raw pose of DWM1001 as a measurement data in KF
                    t_pose_list = [t_pose_x, t_pose_y, t_pose_z]
                    t_pose_xyz = np.array(t_pose_list) # numpy array

                    # t_pose_xyz = np.array([t_pose_x, t_pose_y, t_pose_z])
                    t_pose_xyz.shape = (len(t_pose_xyz), 1)   # force to be a column vector

                    if tag_macID not in self.kalman_list:   # TODO: tag_macID
                        # self.kalman_list.append(tag_id)
                        self.kalman_list.append(tag_macID)
                        # Suppose constant velocity motion model is used (x,y,z and velocities in 3D)
                        A = np.zeros((6,6))
                        H = np.zeros((3, 6))  # measurement (x,y,z without velocities) 

                        self.kalman_list[tag_id] = kf(A, H, tag_macID) # create KF object for tag id
                        # self.kalman_list[tag_id] = kf(A, H, tag_macID) # create KF object for tag id
                        # print(self.kalman_list[tag_id].isKalmanInitialized)
                        
                    if self.kalman_list[tag_id].isKalmanInitialized == False:  
                        # Initialize the Kalman by asigning required parameters
                        # This should be done only once for each tag
                        A, B, H, Q, R, P_0, x_0  = initConstVelocityKF() # for const velocity model
                            
                        self.kalman_list[tag_id].assignSystemParameters(A, B, H, Q, R, P_0, x_0)  # [tag_id]
                        self.kalman_list[tag_id].isKalmanInitialized = True                            
                        # print(self.kalman_list[tag_id].isKalmanInitialized)                           
                   
                    self.kalman_list[tag_id].performKalmanFilter(t_pose_xyz, 0)  
                    t_pose_vel_kf = self.kalman_list[tag_id].x_m  # state vector contains both pose and velocities data
                    t_pose_kf = t_pose_vel_kf[0:3]  # extract only position data (x,y,z)
                    # print(t_pose_kf)                      
                    self.publishTagPoseKF(tag_id, tag_macID, t_pose_kf)
                    # print(len(self.kalman_list))
                        
                    ############### Kalman Filter ###############

                    # except IndexError:
                    # rospy.loginfo("Found index error in the network array!DO SOMETHING!")

                except KeyboardInterrupt:
                    rospy.loginfo("Quitting DWM1001 Shell Mode and closing port, allow 1 second for UWB recovery")
                    # self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
                    # self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

                finally:
                    # rospy.loginfo("Quitting, and sending reset command to dev board")
                    # self.serialPortDWM1001.reset_input_buffer()
                    # self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
                    # self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
                    # self.rate.sleep()
                    # serialReadLine = self.serialPortDWM1001.read_until()
                    # closing text file	
                    # file1.close()
                    if "reset" in serialReadLine:
                        rospy.loginfo("succesfully closed ")
                        self.serialPortDWM1001.close()
        file1.close()
        

    def publishTagPositions(self, serialData):
        """
        Publish anchors and tag in topics using Tag and Anchor Object
        :param networkDataArray:  Array from serial port containing all informations, tag xyz and anchor xyz
        :returns: none
        """ 
        ser_pose_data = [x.strip() for x in serialData.strip().split(',')]

        # If plugged into the active tag:
        if "DIST" in ser_pose_data[0] :

            # The number of elements should be 2 + 6*NUMBER_OF_ANCHORS + 5 (TAG POS)
            # number_of_anchors = int((len(ser_pose_data) - 7)/6)
            number_of_anchors = int(ser_pose_data[1])
     
            # tag_id = str(ser_pose_data[2+6*number_of_anchors], 'UTF8')  # IDs in 0 - 15
            # tag_macID = str(ser_pose_data[2], 'UTF8')
            tag_id = str(ser_pose_data[-5])
            ps = PoseStamped()
            # global ps.pose.position.x
            # global ps.pose.position.y           
            # global ps.pose.position.z            
            ps.pose.position.x = float(ser_pose_data[-4])
            ps.pose.position.y = float(ser_pose_data[-3])
            ps.pose.position.z = float(ser_pose_data[-2])
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0
            ps.header.stamp = rospy.Time.now()   
            ps.header.frame_id = "1AA1" # TODO: From active tag, macID is not published. 

            raw_pose_xzy = [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
            # Assign the PoseStamped msg into CustomTag msg
            tag = CustomTag()
            tag.header = ps.header
            tag.pose_x = ps.pose.position.x
            tag.pose_y = ps.pose.position.y
            tag.pose_z = ps.pose.position.z
            tag.orientation_x = ps.pose.orientation.x
            tag.orientation_y = ps.pose.orientation.y
            tag.orientation_z = ps.pose.orientation.z
            tag.orientation_z = ps.pose.orientation.w
            
            if tag_id not in self.topics:
                self.topics[tag_id] = rospy.Publisher("/dwm1001/id_1AA1/pose", PoseStamped, queue_size=10)
                # self.multipleTags.TagsList.append(tag) # append custom Tags into the multiple tag msgs
                tag_macID = "1AA1"
                          
            # Publish only pose data without "NAN"
            if(np.isnan(raw_pose_xzy).any()): 
                pass
            else:
                self.topics[tag_id].publish(ps) 
                # self.multipleTags.TagsList[int(tag_id)]= 1 # this was "tag"
            self.pub_tags.publish(self.multipleTags)    
        
        # If plugged into passive listener unit:
        elif "POS" in ser_pose_data[0] :
            #rospy.loginfo(ser_pose_data)  # just for debug

            tag_id = str(ser_pose_data[1])  # IDs in 0 - 15
            tag_macID = str(ser_pose_data[2])

            ps = PoseStamped()
            ps.pose.position.x = float(ser_pose_data[3])
            ps.pose.position.y = float(ser_pose_data[4])
            ps.pose.position.z = float(ser_pose_data[5])
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0
            ps.header.stamp = rospy.Time.now()   
            ps.header.frame_id = tag_macID # TODO: Currently, MAC ID of the Tag is set as a frame ID 

            raw_pose_xzy = [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
            # Assign the PoseStamped msg into CustomTag msg
            tag = CustomTag()
            tag.header = ps.header
            tag.pose_x = ps.pose.position.x
            tag.pose_y = ps.pose.position.y
            tag.pose_z = ps.pose.position.z
            tag.orientation_x = ps.pose.orientation.x
            tag.orientation_y = ps.pose.orientation.y
            tag.orientation_z = ps.pose.orientation.z
            tag.orientation_z = ps.pose.orientation.w

            if tag_id not in self.topics:
                self.topics[tag_id] = rospy.Publisher("/dwm1001/id_" + tag_macID + "/pose", PoseStamped, queue_size=10)
                # self.multipleTags.TagsList.append(tag) # append custom Tags into the multiple tag msgs
                           
            # Publish only pose data without "NAN"
            if(np.isnan(raw_pose_xzy).any()): 
                pass
            else:
                self.topics[tag_id].publish(ps) 

    # Publish Tag positions using KF 
    def publishTagPoseKF(self, id_int, tag_macID, kfPoseData):

        ps = PoseStamped()
        ps.pose.position.x = float(kfPoseData[0])
        ps.pose.position.y = float(kfPoseData[1])
        ps.pose.position.z = float(kfPoseData[2])
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0
        ps.header.stamp = rospy.Time.now()   
        ps.header.frame_id = tag_macID # use MAC ID of the Tag as a frame ID for ROS

        if id_int not in self.topics_kf:
            self.topics_kf[id_int] = rospy.Publisher("/dwm1001/id_" + tag_macID + "/pose_kf", PoseStamped, queue_size=10)

        self.topics_kf[id_int].publish(ps)
            
    def initializeDWM1001API(self):
        """
        Initialize dwm1001 api, by sending sending bytes
        :param: returns: none
        """
        # reset incase previuos run didn't close properly
        # self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
        # send ENTER two times in order to access api
        # self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half a second
        # time.sleep(0.5)
        # self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half second
        # time.sleep(0.5)
        # send a third one - just in case
        # self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

def start():
    dwm1001 = dwm1001_localizer()
    dwm1001.main()

if __name__ == '__main__':
    try:
        start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
