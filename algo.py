#!/usr/bin/env python3

import sys
import os
import numpy as np

import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist

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

        # Place code here (remove the pass 
        # statement once you start coding)
         return self.dist[node] + self.h[node]
    
    def solve(self, sn, en):
        open_list = set([])
        closed_list = set([])

        open_list = set([sn])
        while open_list > 0:
            pass

        # Place code here (remove the pass 
        # statement once you start coding)
    
    def reconstruct_path(self,sn,en):
        open_list = set([sn])
        closed_list = set([])

        P = {}
        P[sn] = 0

        par = {}
        par[sn] = sn

        while len(open_list) > 0:
            n = None
            for node in open_list:
                if n == None or P[node] + self.__get_f_score(node) <P[n] + self.__get_f_score(n):
                    n = node;
                if n == None:
                    print('No path exists')
                    return None
                
                if n == en:
                    reconst_path = []

                    while par[n] !=n:
                        reconst_path.append(n)
                        n = par[n]
                    
                    reconst_path.append(sn)
                    reconst_path.reverse()

                    print('Path Found: {}'.format(reconst_path))
                    return reconst_path
                
                for (m, weight) in self.get_neighbors(n):
                    if m not in open_list and m not in closed_list:
                        open_list.add(m)
                        par[m] = n
                        P[n] = P[n] + weight
                    else:
                        if P[m] > P[n] + weight:
                            P[m] = P[n] + weight
                            par[m] = n

                            if m in closed_list:
                                closed_list.remove(m)
                                open_list.add(m)
                open_list.remove(n)
                closed_list.add(n)
        print('Path not found')
        return None