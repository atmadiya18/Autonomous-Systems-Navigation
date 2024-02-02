#!/usr/bin/env python3

import sys
import os
import numpy as np
import yaml
import GoToPose
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from simple_pid import PID
from math import radians, copysign, sqrt, pow, pi, atan2

class Navigation:
    """! Navigation node class.
    This class should server as a template to implement the path planning and 
    path follower components to move the turtlebot from position A to B.
    """
    def __init__(self, node_name='Navigation'):
        """! Class constructor.
        @param  None.
        @return An instance of the Navigation class.
        """
        # ROS related variables
        self.node_name = node_name
        self.rate = 0
        # Path planner/follower related variables
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()


    def init_app(self):
        """! Node intialization.
        @param  None
        @return None.
        """
        # ROS node initilization
        
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)
        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__goal_pose_cbk, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__ttbot_pose_cbk, queue_size=1)
        # Publishers
        self.path_pub = rospy.Publisher('global_plan', Path, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def __goal_pose_cbk(self, data):
        """! Callback to catch the goal pose.
        @param  data    PoseStamped object from RVIZ.
        @return None.
        """
        self.goal_pose = data
        rospy.loginfo('goal_pose:{:.4f},{:.4f}'.format(self.goal_pose.pose.position.x,self.goal_pose.pose.position.y))

    def __ttbot_pose_cbk(self, data):
        """! Callback to catch the position of the vehicle.
        @param  data    PoseWithCovarianceStamped object from amcl.
        @return None.
        """
        # TODO: MAKE SURE YOUR POSITION ESTIMATE IS GOOD ENOUGH.
        self.ttbot_pose = data.pose
        cov = data.pose.covariance
        rospy.loginfo('ttbot_pose:{:.4f},{:.4f}'.format(self.ttbot_pose.pose.position.x,self.ttbot_pose.pose.position.y))
        rospy.loginfo('ttbot_pose:{}'.format(cov))
    
    def a_star_path_planner(self,start_pose,end_pose):
        """! A Start path planner.
        @param  start_pose    PoseStamped object containing the start of the path to be created.
        @param  end_pose      PoseStamped object containing the end of the path to be created.
        @return path          Path object containing the sequence of waypoints of the created path.
        """
        path = Path()
        rospy.loginfo('A* planner.\n> start:{},\n> end:{}'.format(start_pose.pose.position,end_pose.pose.position))
        # TODO: IMPLEMENTATION OF THE ASTAR ALGORITHM
        class AStar():
          def __init__(self,in_tree):
            self.in_tree = in_tree
            self.q = Queue()
            self.dist = {name:np.Inf for name,node in in_tree.g.items()}
            self.h = {name:0 for name,node in in_tree.g.items()}
        
            for name,node in in_tree.g.items():
                start = tuple(map(int, name.split(',')))
                end = tuple(map(int, self.in_tree.end.split(',')))
                self.h[name] = np.sqrt((end[0]-start[0])**2 + (end[1]-start[1])**2)
        
            self.via = {name:0 for name,node in in_tree.g.items()}
            for __,node in in_tree.g.items():
                self.q.push(node)
     
          def __get_f_score(self,node):
                idx = node.name
                return self.dist[idx] + self.h[idx]
    
          def solve(self, sn, en):
                self.dist[sn.name] = 0
                while len(self.q) > 0:
                 self.q.sort(key=self.__get_f_score)
                 u = self.q.pop()
            #print(u.name,self.q.queue)
                 if u.name == en.name:
                    break
                 for i in range(len(u.children)):
                    c = u.children[i]
                    w = u.weight[i]
                    new_dist = self.dist[u.name] + w
                    if new_dist < self.dist[c.name]:
                       self.dist[c.name] = new_dist
                       self.via[c.name] = u.name
    
          def reconstruct_path(self,sn,en):
                start_key = sn.name
                end_key = en.name
                dist = self.dist[end_key]
                u = end_key
                path = [u]
                while u != start_key:
                   u = self.via[u]
                   path.append(u)
                path.reverse()
                return path,dist
        path.poses.append(start_pose)
        path.poses.append(end_pose)
        return path
    
    def get_path_idx(self,path,vehicle_pose):
        """! Path follower.
        @param  path                  Path object containing the sequence of waypoints of the created path.
        @param  current_goal_pose     PoseStamped object containing the current vehicle position.
        @return idx                   Position int the path pointing to the next goal pose to follow.
        """
        idx = 0
        # TODO: IMPLEMENT A MECHANISM TO DECIDE WHICH POINT IN THE PATH TO FOLLOW idx<=len(path)
        
                
        return idx

    def path_follower(self,vehicle_pose, current_goal_pose):
        """! Path follower.
        @param  vehicle_pose           PoseStamped object containing the current vehicle pose.
        @param  current_goal_pose      PoseStamped object containing the current target from the created path. This is different from the global target.
        @return path                   Path object containing the sequence of waypoints of the created path.
        """
        #speed = 0
        #heading = 0
        # TODO: IMPLEMENT PATH FOLLOWER
        if __name__ == '__main__':
            with open("map.yaml",'map') as stream:
                dataMap = yaml.load(stream)
            try:
                rospy.init_node('follow',anonymous = False)
                nav = GoToPose()

                for object in dataMap:
                    if rospy.is_shutdown():
                        break

                    name = object['map']

                    rospy.loginfo("Go to %s pose ", name[:-4])
                    s = nav.goto(object['position'], object['quaternion'])
                    if not s:
                        rospy.loginfo("Failed to reach %s pose", name[:-4])
                        continue
                    rospy.loginfo("Reached %s pose ", name[:-4])

             

    
        return s

    def move_ttbot(self,speed,heading):
        """! Function to move turtlebot passing directly a heading angle and the speed.
        @param  speed     Desired yaw angle.
        @param  heading   Desired speed.
        @return path      object containing the sequence of waypoints of the created path.
        """

        
        def controller()
            pub = rospy.Publisher('Velocity', Twist, queue_size = 10)
            rospy.init_node('controller', anonymous=True)
            r = rospy.Rate(10)
            self.tf_listener = tf.TransformListener()
            self.odom_frame = 'odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        (position, rotation) = self.get_odom()

        last_rotation = 0
        linear_speed = 2    
        angular_speed = 1  


        (goal_x, goal_y, goal_z) = self.getkey()
        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)
    goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        #distance is the error for length, x,y
    distance = goal_distance
    previous_distance = 0
    total_distance = 0

    previous_angle = 0
    total_angle = 0
         
    pid = PID(2, 0.01, 0.05, setpoint=0.5)
    pid.output_limits = (-1, 1)
    cmd_vel = Twist()
    rospy.Subscriber('distance', distance)

    while not rospy.is_shutdown():
        rospy.loginfo('[controller] Running')    
  
        control = pid(distance)
        pub.publish(cmd_vel)
    rospy.loginfo(cmd_vel)
        

        #cmd_vel.linear.x = 0
        #cmd_vel.angular.z = 0

        #self.cmd_vel_pub.publish(cmd_vel)


    def run(self):
        """! Main loop of the node. You need to wait until a new pose is published, create a path and then
        drive the vehicle towards the final pose.
        @param none
        @return none
        """
        
        '''
            Main loop
        '''
        path_complete = False
        timeout = False
        idx = 0
        while not rospy.is_shutdown():
            # 1. Create the path to follow
            path = self.a_star_path_planner(self.ttbot_pose,self.goal_pose)
            # 2. Loop through the path and move the robot
            idx = self.get_path_idx(path,self.ttbot_pose)
            current_goal = path.poses[idx]
            speed,heading = self.path_follower(self.ttbot_pose,current_goal)
            self.move_ttbot(speed,heading)
            # ----------------------------------------
            # TODO: YOU NEED TO ADD YOUR SOLUTION HERE
            # ----------------------------------------
            self.rate.sleep() 
        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.name))


if __name__ == "__main__":
    nav = Navigation(node_name='Navigation')
    nav.init_app()
    try:
        nav.run()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)