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

# @jit(nopython=True)
def two_agent_collision_check(np_arr,agents,bbox):
    if (abs(np_arr[agents[0],0] - np_arr[agents[1],0]) < bbox[0]
    and abs(np_arr[agents[0],1] - np_arr[agents[1],1]) < bbox[1]
    and abs(np_arr[agents[0],2] - np_arr[agents[1],2]) < bbox[2]):
        print("collistion btwn " + str(agents[0]+1) + " and " + str(agents[1]+1))
        return True
    else:
        return False



class CollisionDetector:

    def __init__(self): 

        #Creating goal message to be received by outer loop and considered EmergencyStop.
        self.goal = Goal()
        self.goal.v.x = self.goal.v.y  = 0
        self.goal.v.z = 0
        self.goal.dpsi = 0
        self.goal.p.z = -0.1
        self.goal.power = False
        self.flight_initialized = False
        self.collision_detected = False


        #Retrieving parameters from yaml file. (/param/collision_d.yaml)
        self.bbox_x = rospy.get_param('/bbox_x', 1.0)
        self.bbox_y = rospy.get_param('/bbox_y', 1.0)
        self.bbox_z = rospy.get_param('/bbox_z', 1.0)
        self.is_sim = rospy.get_param('/is_sim', True)
        self.num_of_agents = rospy.get_param('/number_of_agents', 6)

        self.killed_agents = []

        self.initialized = True

        self.state_pos = np.empty([self.num_of_agents,3]) #CHANGE TO AGENT #

    # collision detection


    def collisionDetect(self, timer):
        
        if self.initialized:
            for i in range(1,self.num_of_agents + 1):
                # print("Running..Waiting for Collision")
                for j in range(i+1,self.num_of_agents + 1):
                  
                    agent1 = "SQ0" + str(i) + "s" if self.is_sim else "NX0" + str(i)
                    agent2 = "SQ0" + str(j) + "s" if self.is_sim else "NX0" + str(j)

                    if two_agent_collision_check(self.state_pos,[i-1,j-1],[self.bbox_x,self.bbox_y,self.bbox_y]):
                        self.kill_command(agent1,agent2)
                    # if (abs(self.state_pos[i-1,0] - self.state_pos[j-1,0]) < self.bbox_x
                    #     and abs(self.state_pos[i-1,1] - self.state_pos[j-1,1]) < self.bbox_y
                    #     and abs(self.state_pos[i-1,2] - self.state_pos[j-1,2]) < self.bbox_z):

                                    #### SCREEN OUTPUT
                                    # print("collistion btwn " + agent1 + " and " + agent2)
                                    # print("Fro X diff:"+str(abs(self.state_pos[i-1,0] - self.state_pos[j-1,0])))
                                    # print("Fro Y diff:"+str(abs(self.state_pos[i-1,1] - self.state_pos[j-1,1])))
                                    # print("Fro z diff:"+str(abs(self.state_pos[i-1,2] - self.state_pos[j-1,2])))
                                    # x_diff = abs(self.state_pos[i-1,0] - self.state_pos[j-1,0])
                                    # y_diff = abs(self.state_pos[i-1,1] - self.state_pos[j-1,1])
                                    # z_diff = abs(self.state_pos[i-1,2] - self.state_pos[j-1,2])   
                                    # max_dist = max(x_diff, y_diff, z_diff) 
                                    #### SCREEN OUTPUT


                                    # print("Dist between collided agents is: " + str(max_dist))
                                    # self.kill_command(agent1,agent2)




    def SQ01stateCB(self, data):
        self.state_pos[0,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
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
    def SQ07stateCB(self, data):
        self.state_pos[6,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ08stateCB(self, data):
        self.state_pos[7,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ09stateCB(self, data):
        self.state_pos[8,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ10stateCB(self, data):
        self.state_pos[9,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])





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
        
    rospy.Subscriber("SQ01s/state", State, c.SQ01stateCB)
    rospy.Subscriber("SQ02s/state", State, c.SQ02stateCB)
    rospy.Subscriber("SQ03s/state", State, c.SQ03stateCB)
    rospy.Subscriber("SQ04s/state", State, c.SQ04stateCB)
    rospy.Subscriber("SQ05s/state", State, c.SQ05stateCB)
    rospy.Subscriber("SQ06s/state", State, c.SQ06stateCB)
    rospy.Subscriber("SQ07s/state", State, c.SQ07stateCB)
    rospy.Subscriber("SQ08s/state", State, c.SQ08stateCB)
    rospy.Subscriber("SQ09s/state", State, c.SQ09stateCB)
    rospy.Subscriber("SQ10s/state", State, c.SQ10stateCB)
    # rospy.Subscriber("SQ01s/state", State, c.SQ01stateCB)
    rospy.Timer(rospy.Duration(0.001), c.collisionDetect)
    rospy.spin()

if __name__ == '__main__':
    
    rospy.init_node('CollisionDetector')
    rate = rospy.Rate(100) # ROS Rate at 5Hz
    startNode()


######TODO
#-Wait for Goal to be publised to start algorithm (No?)
