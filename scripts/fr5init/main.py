#!/usr/bin/env python
# -*- coding: utf-8 -*-

#这是使用法奥api进行路径规划的demo，可以直接运行

import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)


import rospy
import time
from math import sin, cos,pi
from chemistryexp import HNchemistry
import numpy
##############################预设全局变量##################################
'''
常用变量
'''
eP1=[0.000,0.000,0.000,0.000]
dP1=[1.000,1.000,1.000,1.000,1.000,1.000]
oP1=[0.000,0.000,0.000,0.000,0.000,0.000]
a_bias = 220.0
b_bias = 150.0
'''
仪器区的初始位置
'''
liangtong_xy_input = [600, 0] # 用于称量液体浓盐酸的量筒，对于B
shaobei_xy_input = [600, -100] # 用于称量固体C的烧杯，对于B
sanjing_xy_input = [520, -300] # 用于反应容器的三颈烧瓶，对于B
liangtong_xy_output = [-500, -100]
shaobei_xy_output = [-500, -200]



'''
关键姿势的J与P，做伺服到位用
'''
j1_auto_weight = [-36.870, -59.151, -137.756, -163.093, -126.870, 0.000]
j2_auto_weight = [40.542, -57.601, -138.394, -164.005, -49.458, 0.000]
j3_auto_weight = [-64.941, -59.151, -137.756, -163.094, 25.056,0.001]
j4_auto_weight = [-49.458, -55.125, -124.682, -180.193, -49.458, 0.000]# 这个是机械臂复位的J
j5_auto_weight = [-119.608, -101.188, -110.403, -148.409, -119.608, 0.000]# 这是机械臂A要靠过来抓三颈烧瓶时的先验关节角度

p6_auto_weight = [320.0, -340.0, 400.0, 90.0, 0.0 ,0.0]

# p1_auto_weight = []
# fr5_A = HNchemistry(1)
# fr5_B = HNchemistry(2)

def init():
    global fr5_A
    global fr5_B
    fr5_A = HNchemistry(1)
    fr5_B = HNchemistry(2)
    # time.sleep(1)
    fr5_A.dou_go_start(fr5_B)




if __name__ == "__main__":
    #############################开始###############################
    print("---------------FR5机械臂化学协作实验------------------\n") 
    init()
    ###################FR5B去仪器区分别抓取试管和烧杯########################
    fr5_A.Auto_Weight(fr5_B)
    

    # fr5_A.dou_go_start(fr5_B)
    # array = [550, 0]
    # string = ' '.join(map(str, array))
    # fr5_B.F101_catch_02(string, "xp" , "2" ,False)

    