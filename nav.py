#!/usr/bin/env python3
import sys
import os
import GoToPose
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from simple_pid import PID
from math import radians, copysign, sqrt, pow, pi, atan2
from PIL import Image, ImageOps 
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import yaml
import pandas as pd
from copy import copy, deepcopy
import time

# urla = 'https://github.com/KC-git-usr/Autonomous_Systems/blob/main/catkin_ws/src/lab_4_pkg/maps/map.pgm'
# df1 = pd.read_csv(urla)

from PIL import Image, ImageOps 
import numpy as np
from copy import copy, deepcopy
import time
from graphviz import Graph

#from nav_msgs.msg import Path

class Navigation:
    def __init__(self, node_name='Navigation'):
        #Class constructor.
        # ROS related variables
        self.node_name = node_name
        self.rate = 0
        # Path planner/follower related variables
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()
    def init_app(self):
        #ROS node initilization
        
         rospy.init_node(self.node_name, anonymous=True)
         self.rate = rospy.Rate(10)
        # Subscribers
         rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__goal_pose_cbk, queue_size=1)
         rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__ttbot_pose_cbk, queue_size=1)
        # Publishers
         self.path_pub = rospy.Publisher('global_plan', Path, queue_size=10)
         self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def __goal_pose_cbk(self, data):
        """! Callback to catch the goal pose.
        @param  data    PoseStamped object from RVIZ.
        @return None.
        """
        self.goal_pose = data
        rospy.loginfo('goal_pose:{:.4f},{:.4f}'.format(self.goal_pose.pose.position.x,self.goal_pose.pose.position.y))
    def __ttbot_pose_cbk(self, data):
        self.ttbot_pose = data.pose
        cov = data.pose.covariance
        rospy.loginfo('ttbot_pose:{:.4f},{:.4f}'.format(self.ttbot_pose.pose.position.x,self.ttbot_pose.pose.position.y))
        rospy.loginfo('ttbot_pose:{}'.format(cov))
    
    def a_star_path_planner(self,start_pose,end_pose):
        path = Path()
        rospy.loginfo('A* planner.\n> start:{},\n> end:{}'.format(start_pose.pose.position,end_pose.pose.position))



class Map():
    def __init__(self, map_name):
        self.map_im, self.map_df, self.limits = self.__open_map(map_name)
        self.image_array = self.__get_obstacle_map(self.map_im, self.map_df)
    
    def __repr__(self):
        fig, ax = plt.subplots(dpi=150)
        ax.imshow(self.image_array,extent=self.limits, cmap=cm.gray)
        ax.plot()
        return ""
        
    def __open_map(self,map_name):
        # Open the YAML file which contains the map name and other
        # configuration parameters
        f = open(map_name + '.yaml', 'r')
        map_df = pd.json_normalize(yaml.safe_load(f))
        # Open the map image
        map_name = map_df.image[0]
        im = Image.open(map_name)
        size = 200, 200
        im.thumbnail(size)
        im = ImageOps.grayscale(im)
        # Get the limits of the map. This will help to display the map
        # with the correct axis ticks.
        xmin = map_df.origin[0][0]
        xmax = map_df.origin[0][0] + im.size[0] * map_df.resolution[0]
        ymin = map_df.origin[0][1]
        ymax = map_df.origin[0][1] + im.size[1] * map_df.resolution[0]

        return im, map_df, [xmin,xmax,ymin,ymax]

    def __get_obstacle_map(self,map_im, map_df):
        img_array = np.reshape(list(self.map_im.getdata()),(self.map_im.size[1],self.map_im.size[0]))
        up_thresh = self.map_df.occupied_thresh[0]*255
        low_thresh = self.map_df.free_thresh[0]*255

        for j in range(self.map_im.size[0]):
            for i in range(self.map_im.size[1]):
                if img_array[i,j] > up_thresh:
                    img_array[i,j] = 255
                else:
                    img_array[i,j] = 0
        return img_array


print(Map('map'))


class MapProcessor():
    def __init__(self,name):
        self.map = Map(name)
        self.inf_map_img_array = np.zeros(self.map.image_array.shape)
        self.map_graph = Tree(name)
    
    def __modify_map_pixel(self,map_array,i,j,value,absolute):
        if( (i >= 0) and 
            (i < map_array.shape[0]) and 
            (j >= 0) and
            (j < map_array.shape[1]) ):
            if absolute:
                map_array[i][j] = value
            else:
                map_array[i][j] += value 
    
    def __inflate_obstacle(self,kernel,map_array,i,j,absolute):
        dx = int(kernel.shape[0]//2)
        dy = int(kernel.shape[1]//2)
        if (dx == 0) and (dy == 0):
            self.__modify_map_pixel(map_array,i,j,kernel[0][0],absolute)
        else:
            for k in range(i-dx,i+dx):
                for l in range(j-dy,j+dy):
                    self.__modify_map_pixel(map_array,k,l,kernel[k-i+dx][l-j+dy],absolute)
        
    def inflate_map(self,kernel,absolute=True):
        # Perform an operation like dilation, such that the small wall found during the mapping process
        # are increased in size, thus forcing a safer path.
        self.inf_map_img_array = np.zeros(self.map.image_array.shape)
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.map.image_array[i][j] == 0:
                    self.__inflate_obstacle(kernel,self.inf_map_img_array,i,j,absolute)
        r = np.max(self.inf_map_img_array)-np.min(self.inf_map_img_array)
        if r == 0:
            r = 1
        self.inf_map_img_array = (self.inf_map_img_array - np.min(self.inf_map_img_array))/r
                
    def get_graph_from_map(self):
        # Create the nodes that will be part of the graph, considering only valid nodes or the free space
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:
                    node = Node('%d,%d'%(i,j))
                    self.map_graph.add_node(node)
        # Connect the nodes through edges
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:                    
                    if (i > 0):
                        if self.inf_map_img_array[i-1][j] == 0:
                            # add an edge up
                            child_up = self.map_graph.g['%d,%d'%(i-1,j)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up],[1])
                    if (i < (self.map.image_array.shape[0] - 1)):
                        if self.inf_map_img_array[i+1][j] == 0:
                            # add an edge down
                            child_dw = self.map_graph.g['%d,%d'%(i+1,j)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw],[1])
                    if (j > 0):
                        if self.inf_map_img_array[i][j-1] == 0:
                            # add an edge to the left
                            child_lf = self.map_graph.g['%d,%d'%(i,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_lf],[1])
                    if (j < (self.map.image_array.shape[1] - 1)):
                        if self.inf_map_img_array[i][j+1] == 0:
                            # add an edge to the right
                            child_rg = self.map_graph.g['%d,%d'%(i,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_rg],[1])
                    if ((i > 0) and (j > 0)):
                        if self.inf_map_img_array[i-1][j-1] == 0:
                            # add an edge up-left 
                            child_up_lf = self.map_graph.g['%d,%d'%(i-1,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_lf],[np.sqrt(2)])
                    if ((i > 0) and (j < (self.map.image_array.shape[1] - 1))):
                        if self.inf_map_img_array[i-1][j+1] == 0:
                            # add an edge up-right
                            child_up_rg = self.map_graph.g['%d,%d'%(i-1,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_rg],[np.sqrt(2)])
                    if ((i < (self.map.image_array.shape[0] - 1)) and (j > 0)):
                        if self.inf_map_img_array[i+1][j-1] == 0:
                            # add an edge down-left 
                            child_dw_lf = self.map_graph.g['%d,%d'%(i+1,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_lf],[np.sqrt(2)])
                    if ((i < (self.map.image_array.shape[0] - 1)) and (j < (self.map.image_array.shape[1] - 1))):
                        if self.inf_map_img_array[i+1][j+1] == 0:
                            # add an edge down-right
                            child_dw_rg = self.map_graph.g['%d,%d'%(i+1,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_rg],[np.sqrt(2)])                    
        
    def gaussian_kernel(self, size, sigma=1):
        size = int(size) // 2
        x, y = np.mgrid[-size:size+1, -size:size+1]
        normal = 1 / (2.0 * np.pi * sigma**2)
        g =  np.exp(-((x**2 + y**2) / (2.0*sigma**2))) * normal
        r = np.max(g)-np.min(g)
        sm = (g - np.min(g))*1/r
        return sm
    
    def rect_kernel(self, size, value):
        m = np.ones(shape=(size,size))
        return m
    
    def draw_path(self,path):
        path_tuple_list = []
        path_array = copy(self.inf_map_img_array)
        for idx in path:
            tup = tuple(map(int, idx.split(',')))
            path_tuple_list.append(tup)
            path_array[tup] = 0.5
        return path_array


g = Graph('G')

g.node('a','a',style='filled')
g.node('b','b')
g.node('c','c')
g.node('d','d')
g.node('e','e')

g.edge('a','b',shape='none')
g.edge('a','c')
g.edge('c','d')
g.edge('c','e')
    



class Queue():
    def __init__(self, init_queue = []):
        self.queue = copy(init_queue)
        self.start = 0
        self.end = len(self.queue)-1
    
    def __len__(self):
        numel = len(self.queue)
        return numel
    
    def __repr__(self):
        q = self.queue
        tmpstr = ""
        for i in range(len(self.queue)):
            flag = False
            if(i == self.start):
                tmpstr += "<"
                flag = True
            if(i == self.end):
                tmpstr += ">"
                flag = True
            
            if(flag):
                tmpstr += '| ' + str(q[i]) + '|\n'
            else:
                tmpstr += ' | ' + str(q[i]) + '|\n'
            
        return tmpstr
    
    def __call__(self):
        return self.queue
    
    def initialize_queue(self,init_queue = []):
        self.queue = copy(init_queue)
    
    def sort(self,key=str.lower):
        self.queue = sorted(self.queue,key=key)
        
    def push(self,data):
        self.queue.append(data)
        self.end += 1
    
    def pop(self):
        p = self.queue.pop(self.start)
        self.end = len(self.queue)-1
        return p
    
class Node():
    def __init__(self,name):
        self.name = name
        self.children = []
        self.weight = []
        
    def __repr__(self):
        return self.name
        
    def add_children(self,node,w=None):
        if w == None:
            w = [1]*len(node)
        self.children.extend(node)
        self.weight.extend(w)
    
class Tree():
    def __init__(self,name):
        self.name = name
        self.root = 0
        self.end = 0
        self.g = {}
        self.g_visual = Graph('G')
    
    def __call__(self):
        for name,node in self.g.items():
            if(self.root == name):
                self.g_visual.node(name,name,color='red')
            elif(self.end == name):
                self.g_visual.node(name,name,color='blue')
            else:
                self.g_visual.node(name,name)
            for i in range(len(node.children)):
                c = node.children[i]
                w = node.weight[i]
                #print('%s -> %s'%(name,c.name))
                if w == 0:
                    self.g_visual.edge(name,c.name)
                else:
                    self.g_visual.edge(name,c.name,label=str(w))
        return self.g_visual
    
    def add_node(self, node, start = False, end = False):
        self.g[node.name] = node
        if(start):
            self.root = node.name
        elif(end):
            self.end = node.name
            
    def set_as_root(self,node):
        # These are exclusive conditions
        self.root = True
        self.end = False
    
    def set_as_end(self,node):
        # These are exclusive conditions
        self.root = False
        self.end = True   


    
a = Node('a')
b = Node('b')
c = Node('c')
d = Node('d')
e = Node('e')
f = Node('f')

a.add_children([c],[1])
b.add_children([c,e],[1,1])
c.add_children([b,e,d],[1,3,1])
e.add_children([b,c],[1,3])
d.add_children([c],[1])

tree = Tree('tree')

tree.add_node(a,start=True)
tree.add_node(b)
tree.add_node(c)
tree.add_node(d)
tree.add_node(e,end=True)
tree.add_node(f)


class Astar():
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

mp = MapProcessor('map')

kr = mp.rect_kernel(5,1)
mp.inflate_map(kr,True)

mp.get_graph_from_map()


start_pt = "95,95"
end_pt = "82,122" # goal 

mp.map_graph.root = start_pt
mp.map_graph.end = end_pt

as_maze = Astar(mp.map_graph) # creates object

as_maze.solve(mp.map_graph.g[mp.map_graph.root],mp.map_graph.g[mp.map_graph.end]) # solves map 


path_as,dist_as = as_maze.reconstruct_path(mp.map_graph.g[mp.map_graph.root],mp.map_graph.g[mp.map_graph.end]) # finds path to goal 

print(path_as)
print(dist_as)

def path_follower(self):
    if __name__ == '__main__':
        with open("map.yaml",'map') as stream:
            dataMap = yaml.load(stream)
        try:
            rospy.init_node('follow',anonymous = False)
            self.nav = GoToPose()

            for object in dataMap:
                if rospy.is_shutdown():
                    break

                self.name = object['map']

                rospy.loginfo("Go to %s pose ", self.name[:-4])
                self.s = self.nav.goto(object['position'], object['quaternion'])
                if not self.s:
                    rospy.loginfo("Failed to reach %s pose", self.name[:-4])
                    continue
                rospy.loginfo("Reached %s pose ",self.name[:-4])

        return self.s
def controller(self):
    pub = rospy.Publisher('Velocity', Twist, queue_size = 10)
    rospy.init_node('controller', anonymous=True)
    self.r = rospy.Rate(10)
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
    (position) = self.get_odom()  
    (goal_x, goal_y, goal_z) = self.getkey()
    if goal_z > 180 or goal_z < -180:
        print("you input wrong z range.")
        self.shutdown()
    goal_z = np.deg2rad(goal_z)
    goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        #distance is the error for length, x,y
    distance = goal_distance
         
    pid = PID(2, 0.01, 0.05, setpoint=0.5)
    pid.output_limits = (-1, 1)
    cmd_vel = Twist()
    rospy.Subscriber('distance', distance)

    while not rospy.is_shutdown():
        rospy.loginfo('[controller] Running')    
        self.control = pid(distance)
        pub.publish(cmd_vel)
    rospy.loginfo(cmd_vel)
if __name__ == "__main__":
    nav = Navigation(node_name='Navigation')
    nav.init_app()
    try:
        nav.path_follower()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
        









