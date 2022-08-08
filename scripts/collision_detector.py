#!/usr/bin/env python
  
#Author:
#Date: 

#you can get transformation btwn two agents and if they are too close (violating bbox) then it prints out warning
#the reason why we use tf instead of snapstack_msgs/State is two agents publish their states asynchrnonously and therefore comparing
#these states (with slightly different timestamp) is not accurate position comparison. Whereas tf always compares two states with the same time stamp

###
import subprocess
import rosnode
####
import math
import os
import time
import rospy
import rosgraph
from geometry_msgs.msg import PoseStamped
from snapstack_msgs.msg import State , Goal
import numpy as np
from random import *
import tf2_ros

from numba import jit
from numba.experimental import jitclass




# class Goal:
#     def __init__(self):
#         self.power = true






class CollisionDetector:

    def __init__(self): 

        #####   TF NOT NEEDED ANYMORE
        # self.tfBuffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # rospy.sleep(3) #Important, if not it won't work
        #####   TF NOT NEEDED ANYMORE





        # tolerance
        self.tol = 0.07



        ##### TESTS!!!!
        ## CREATION 0 GOAL

        #GOAL
        self.goal = Goal()
        self.goal.v.x = self.goal.v.y  = 0
        self.goal.v.z = 0
        self.goal.dpsi = 0
        self.goal.p.z = -0.1
        self.goal.power = False
        self.flight_initialized = False
        self.collision_detected = False

        ##testing pubs
        # self.pub_goal1 = rospy.Publisher('/SQ01s' +'/collision_detector', Goal, queue_size=10)
        # self.pub_goal2 = rospy.Publisher('/SQ02s' +'/collision_detector', Goal, queue_size=10)



        #### TESTS!!!!!

        # bbox size
        #self.bbox_x = rospy.get_param('bbox_x', 1.0) - self.tol #default value is 1.0 
        #self.bbox_y = rospy.get_param('bbox_y', 1.0) - self.tol #default value is 1.0 
        #self.bbox_z = rospy.get_param('bbox_z', 1.5) - self.tol #default value is 1.5
        self.bbox_x = rospy.get_param('/bbox_x', 1.0)
        self.bbox_y = rospy.get_param('/bbox_y', 1.0)
        self.bbox_z = rospy.get_param('/bbox_z', 1.0)
        self.is_sim = rospy.get_param('/is_sim', True)
        self.num_of_agents = rospy.get_param('/number_of_agents', 6)

        self.killed_agents = []

        self.initialized = True

        self.state_pos = np.empty([6,3]) #CHANGE TO AGENT #

    # collision detection


    def collisionDetect(self, timer):
        
        if self.initialized:
            # ###TEST CODE
            # agent1 = "SQ0" + '1' + "s" if self.is_sim else "NX0" + str(i)
            # agent2 = "SQ0" + '2' + "s" if self.is_sim else "NX0" + str(j)
            # trans = self.get_transformation(agent1, agent2)
                  
            # if trans is not None:    
            #     if (abs(trans.transform.translation.x) < self.bbox_x
            #         and abs(trans.transform.translation.y) < self.bbox_y
            #         and abs(trans.transform.translation.z) < self.bbox_z):
            #         self.kill_command(trans.header.frame_id,trans.child_frame_id)
            ####  


            for i in range(1,self.num_of_agents + 1):
                # print("Running..Waiting for Collision")
                for j in range(i+1,self.num_of_agents + 1):
                  
                    agent1 = "SQ0" + str(i) + "s" if self.is_sim else "NX0" + str(i)
                    agent2 = "SQ0" + str(j) + "s" if self.is_sim else "NX0" + str(j)

                    ###### NOT USING TF ANYMORE
                    # trans = self.get_transformation(agent1, agent2)

                    # if trans is not None:
                    
                    #     if (abs(trans.transform.translation.x) < self.bbox_x
                    #         and abs(trans.transform.translation.y) < self.bbox_y
                    #         and abs(trans.transform.translation.z) < self.bbox_z):
                            
                    #         # print("collistion btwn " + trans.header.frame_id + " and " + trans.child_frame_id)

                    #         # max_dist = max(abs(trans.transform.translation.x), abs(trans.transform.translation.y), abs(trans.transform.translation.z))

                    #         # print("violation dist is " + str(max_dist))
                            
                    #         self.kill_command(trans.header.frame_id,trans.child_frame_id)
                    ###### NOT USING TF ANYMORE

                    if (abs(self.state_pos[i-1,0] - self.state_pos[j-1,0]) < self.bbox_x
                        and abs(self.state_pos[i-1,1] - self.state_pos[j-1,1]) < self.bbox_y
                        and abs(self.state_pos[i-1,2] - self.state_pos[j-1,2]) < self.bbox_z):
                                    print("collistion btwn " + agent1 + " and " + agent2)
                                    print("Fro X diff:"+str(abs(self.state_pos[i-1,0] - self.state_pos[j-1,0])))
                                    print("Fro Y diff:"+str(abs(self.state_pos[i-1,1] - self.state_pos[j-1,1])))
                                    print("Fro z diff:"+str(abs(self.state_pos[i-1,2] - self.state_pos[j-1,2])))
                                    x_diff = abs(self.state_pos[i-1,0] - self.state_pos[j-1,0])
                                    y_diff = abs(self.state_pos[i-1,1] - self.state_pos[j-1,1])
                                    z_diff = abs(self.state_pos[i-1,2] - self.state_pos[j-1,2])   
                                    max_dist = max(x_diff, y_diff, z_diff) 
                                    print("Dist between collided agents is: " + str(max_dist))
                                    self.kill_command(agent1,agent2)




    def SQ01stateCB(self, data):
        self.state_pos[0,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
        # self.initialized = True
    def SQ02stateCB(self, data):
        self.state_pos[1,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ03stateCB(self, data):
        self.state_pos[2,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ04stateCB(self, data):
        self.state_pos[3,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ05stateCB(self, data):
        self.state_pos[4,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ06stateCB(self, data):
        self.state_pos[5,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
 

     ###### NOT USING TF ANYMORE
    # def get_transformation(self, source_frame, target_frame):

    #     # get the tf at first available time
    #     try:
    #         transformation = self.tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time(0), rospy.Duration(0.001))
    #         return transformation
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         pass
    #         # rospy.logerr("Unable to find the transformation")
     ###### NOT USING TF ANYMORE

    def kill_command(self,agent1, agent2): 
        # print(agent1 not in self.killed_agents)
        # print(self.killed_agents)
        # if agent1 not in self.killed_agents:
        self.pub_goal1 = rospy.Publisher('/'+ agent1 +'/collision_detector', Goal, queue_size=10)
        self.pub_goal1.publish(self.goal)
        # self.killed_agents.append(agent1)
        print(str(agent1) + 'has been killed')   

        # if agent2 not in self.killed_agents: 
        self.pub_goal2 = rospy.Publisher('/'+ agent2 +'/collision_detector', Goal, queue_size=10)
        self.pub_goal2.publish(self.goal)
        # self.killed_agents.append(agent2)
        print(str(agent2) + 'has been killed') 
        # print(self.killed_agents)

        # pass
    

def startNode():
    c = CollisionDetector()

    

    is_sim = c.is_sim
    agent_num = c.num_of_agents + 1
    agent_name_list = []

    if is_sim:
        for i in range(1,agent_num):
            agent_name_list.append("SQ0"+ str(i)+"s")

    else:
        for i in range(1,agent_num):
            agent_name_list.append("NX0" + str(i))
        
    rospy.Subscriber(agent_name_list[0]+ "SQ01s/state", State, c.SQ01stateCB)
    rospy.Subscriber(agent_name_list[1]+ "SQ02s/state", State, c.SQ02stateCB)
    rospy.Subscriber(agent_name_list[2]+ "SQ03s/state", State, c.SQ03stateCB)
    rospy.Subscriber(agent_name_list[3]+ "SQ04s/state", State, c.SQ04stateCB)
    rospy.Subscriber(agent_name_list[4]+ "SQ05s/state", State, c.SQ05stateCB)
    rospy.Subscriber(agent_name_list[5]+ "SQ06s/state", State, c.SQ06stateCB)
    # rospy.Subscriber(agent_name_list[6]+ "/state", State, c.SQ07stateCB)
    # rospy.Subscriber(agent_name_list[7]+ "/state", State, c.SQ08stateCB)
    # rospy.Subscriber(agent_name_list[8]+ "/state", State, c.SQ09stateCB)
    # rospy.Subscriber("SQ10s/state", State, c.SQ10stateCB)
    # rospy.Subscriber("SQ01s/state", State, c.SQ01stateCB)
    rospy.Timer(rospy.Duration(0.001), c.collisionDetect)
    rospy.spin()

if __name__ == '__main__':
    
    rospy.init_node('CollisionDetector')
    rate = rospy.Rate(1) # ROS Rate at 5Hz
    startNode()


######TODO
#-Wait for Goal to be publised to start algorithm
