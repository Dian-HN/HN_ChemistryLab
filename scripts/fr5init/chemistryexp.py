#coding:UTF-8
'''
    HN代料模拟实验，包含六个流程的实现
    环境：Pytohn3.10 + ROS Noetic + Ubuntu20.04
    机器人：FR5
    创建日期：2024.04.16
    更新日期：2024.04.16,添加了化学实验的类，完成了基本骨架，zjh
            2024.04.16，xxxxxx，xxx
            2024.04.16，xxxxxx，xxx
            2024.04.16，xxxxxx，xxx
            2024.04.16，xxxxxx，xxx
            2024.04.16，xxxxxx，xxx
            ...

'''
import sys
import os
sys.path.append('../')
# 获取parent_folder文件夹的路径
parent_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# 将parent_folder文件夹添加到sys.path列表中
sys.path.append(parent_path)
from fr5_init import fr5robot
import frrpc
import time
import copy
import math
import numpy as np
import threading
import rospy

class HNchemistry(fr5robot):
    def __init__(self, index=1):
        super().__init__(index)

    def Add_Solid_Liquid(self):
        '''
        固液加料实验
        '''
        
        pass

    def Mix_Solid_Liquid(self):
        '''
        固液混合实验
        '''
        pass

    def Add_KMnO4(self):
        '''
        添加KMnO4
        '''
        pass

    def Add_H2O2(self):
        '''
        添加H2O2
        '''
        pass

    def Wash(self):
        '''
        清洗
        '''
        pass
    
    def Seprarte(self):
        '''
        产物分离
        '''
        pass
    
