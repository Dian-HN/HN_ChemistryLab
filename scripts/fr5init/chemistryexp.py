#coding:UTF-8
'''
    HN代料模拟实验，包含六个流程的实现
    环境：Pytohn3.10 + ROS Noetic + Ubuntu20.04
    代码规范：变量为小写字母加下划线，函数为大写字母加下划线
    机器人：FR5
    创建日期：2024.04.16
    更新日期：2024.04.16,添加了化学实验的类，完成了基本骨架，zjh
            2024.04.29，补充了缺失的各个独立单元的函数体，zjh
            2024.04.29，提供了定点抓取，定点放置函数，自动称量函数写了一半，zjh
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
import time
import copy
import math
import numpy as np
import threading
import rospy
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
liangtong_xy_input = [600, -100] # 用于称量液体浓盐酸的量筒，对于B
shaobei_xy_input = [600, -200] # 用于称量固体C的烧杯，对于B
sanjing_xy_input = [600, 100] # 用于反应容器的三颈烧瓶，对于B
liangtong_xy_output = [-500, -100]
shaobei_xy_output = [-500, -200]



'''
关键姿势的J与P，做伺服到位用
'''
j1_auto_weight = [-36.870, -59.151, -137.756, -163.093, -126.870, 0.000]
j2_auto_weight = [40.542, -57.601, -138.394, -164.005, -49.458, 0.000]
j3_auto_weight = [-64.941, -59.151, -137.756, -163.094, 25.056,0.001]
# p1_auto_weight = []





class HNchemistry(fr5robot):
    def __init__(self, index=1):
        super().__init__(index)

    def F101_catch_02(self,start_catch_position,catch_direction,sel_num,is_force_sensor = False,is_display = False):
     
     '''
    指定坐标抓取指定物体
    预期效果： 机械臂回到起始区————运动到指定位置抓取物体————抬起展示抓取效果————松开夹爪回到结束区
        start_catch_position: 目标物体绝对xy坐标
        catch_direction: ym——y轴负方向  xm——x负方向
        sel_num:抓取的对: 1---试管 2---烧杯 3---量筒 4---反应瓶
        is_force_sensor:末端是否有力传感器
        is_display:是否为演示抓取，如果是，机械臂会自动复位
     '''
    # 输入坐标
    # start_catch_position = input("--------------请输入目标物体绝对xy坐标---------------\n")
    # 数据处理str转float
     if is_force_sensor == True:
         real_bias = a_bias
     else:
         real_bias = b_bias 
     print("目标物体绝对xy坐标:",start_catch_position)
     start_catch_position = start_catch_position.split(" ")
     start_catch_position = [float(num) for num in start_catch_position]
     # 输入方向
     rxryrz = []
     if catch_direction == "yn":
         print("夹爪夹取方向是：y轴负方向")
         # 数据处理y
         start_catch_position[1] = start_catch_position[1] + real_bias
         start_catch_position[0] = start_catch_position[0]
         rxryrz = [90.0, 0.0, 0.0]
         dir = "yn"
     elif catch_direction == "xn":
         print("夹爪夹取方向是：x轴负方向")
         # 数据处理x
         start_catch_position[0] = start_catch_position[0] + real_bias
         rxryrz = [90.0, 0.0, -90.0]
         dir = "xn"

     elif catch_direction == "xp":
         print("夹爪夹取方向是：x轴正方向")
         # 数据处理x
         start_catch_position[0] = start_catch_position[0] - real_bias
         rxryrz = [90.0, 0.0, 90.0]
         dir = "xp"
     else:
         print("------error!-------")
         exit()
     # 合法检测
     if len(start_catch_position) != 2:
         print("xy坐标错误")
         exit()
     else:
         print("----------------------")
     # 选择抓取对象
     # 注意 抓反应瓶时候，末端离瓶子中心点位180mm 改变瓶子放置
     while True:
         # sel_num = input(
         #     "--------请选择要抓取的对象：输入1---试管 输入2---烧杯 输入3---量筒 输入4---反应瓶--------\n"
         # )
         if int(sel_num) == 1:
             print("抓取的对象：试管")
             start_catch_position += [130.0]
             start_catch_position += rxryrz
             break
         elif int(sel_num) == 2:
             print("抓取的对象：烧杯")
             start_catch_position += [65.0]
             start_catch_position += rxryrz
             break
         elif int(sel_num) == 3:
             print("抓取的对象：量筒")
             start_catch_position += [65.0]
             start_catch_position += rxryrz
             break
         elif int(sel_num) == 4:
             print("抓取的对象：反应瓶")
             start_catch_position += [100.0]
             start_catch_position += rxryrz
             break
         else:
             print("--------输入错误！请重新输入！---------")
             continue
    #  self.Go_to_start_zone()
     if(int(sel_num)==4):
             self.MoveGripper(1, 50, 50, 10, 10000, 1)  # close
             time.sleep(3)
     print('==========================',start_catch_position)
    #  self.Safe_move(start_catch_position, dir)  # for display
     if catch_direction == "yn":
        # y方向留100距离 z方向留200距离
        position1 = start_catch_position
        position1[1] = position1[1] + 100
        position1[2] = position1[2] + 200
        self.robot.MoveCart(position1,0,0)
        time.sleep(2)
        self.MoveL(0.0,0.0,-200.0)
        time.sleep(1)
        self.MoveL(0.0,-100.0,0.0)
        time.sleep(1)
     elif catch_direction == "xn":
         # x方向留100距离 z方向留200距离
        position1 = start_catch_position
        position1[0] = position1[0] + 100
        position1[2] = position1[2] + 200
        self.robot.MoveCart(position1,0,0)
        time.sleep(2)
        self.MoveL(0.0,0.0,-200.0)
        time.sleep(1)
        self.MoveL(-100,0.0,0.0)
        time.sleep(1)
     elif catch_direction == "xp":
         # x方向留-100距离 z方向留200距离
        position1 = start_catch_position
        position1[0] = position1[0] - 100
        position1[2] = position1[2] + 200
        self.robot.MoveCart(position1,0,0)
        time.sleep(2)
        self.MoveL(0.0,0.0,-200.0)
        time.sleep(1)
        self.MoveL(100,0.0,0.0)
        time.sleep(1)

     time.sleep(1)
     # 拿起+放下
     
     if(int(sel_num)==4):
        self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close
        time.sleep(3)
     else:
        self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close
        time.sleep(3)
        
     if(is_display):
        self.MoveL(0.0,0.0,20.0,20.0)
        time.sleep(1)
        self.MoveL(0.0,0.0,-20.0,20.0)
        time.sleep(1)
        self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
        time.sleep(3)
        self.Go_to_start_zone()
     else:
        self.MoveL(0.0,0.0,20.0,20.0)
        time.sleep(1) 
     print("动作完成")

    def F101_put_01(self,start_catch_position,catch_direction,sel_num,is_force_sensor = False,is_display = False):
     
     '''
    指定坐标放置指定物体
    预期效果： 机械臂回到起始区————运动到指定位置抓取物体————抬起展示抓取效果————松开夹爪回到结束区
        start_catch_position: 目标物体绝对xy坐标
        catch_direction: ym——y轴负方向  xm——x负方向
        sel_num:抓取的对: 1---试管 2---烧杯 3---量筒 4---反应瓶
        is_force_sensor:末端是否有力传感器
        is_display:是否为演示，如果是，机械臂会自动复位
     '''
    # 输入坐标
    # start_catch_position = input("--------------请输入目标物体绝对xy坐标---------------\n")
    # 数据处理str转float
     if is_force_sensor == True:
         real_bias = a_bias
     else:
         real_bias = b_bias 
     print("目标物体绝对xy坐标:",start_catch_position)
     start_catch_position = start_catch_position.split(" ")
     start_catch_position = [float(num) for num in start_catch_position]
     # 输入方向
     rxryrz = []
     if catch_direction == "yn":
         print("夹爪夹取方向是：y轴负方向")
         # 数据处理y
         start_catch_position[1] = start_catch_position[1] + a_bias
         start_catch_position[0] = start_catch_position[0]
         rxryrz = [90.0, 0.0, 0.0]
         dir = "yn"
     elif catch_direction == "xn":
         print("夹爪夹取方向是：x轴负方向")
         # 数据处理x
         start_catch_position[0] = start_catch_position[0] + a_bias
         rxryrz = [90.0, 0.0, -90.0]
         dir = "xn"

     elif catch_direction == "xp":
         print("夹爪夹取方向是：x轴正方向")
         # 数据处理x
         start_catch_position[0] = start_catch_position[0] - a_bias
         rxryrz = [90.0, 0.0, 90.0]
         dir = "xp"
     else:
         print("------error!-------")
         exit()
     # 合法检测
     if len(start_catch_position) != 2:
         print("xy坐标错误")
         exit()
     else:
         print("----------------------")
     # 选择抓取对象
     # 注意 抓反应瓶时候，末端离瓶子中心点位180mm 改变瓶子放置
     while True:
         # sel_num = input(
         #     "--------请选择要抓取的对象：输入1---试管 输入2---烧杯 输入3---量筒 输入4---反应瓶--------\n"
         # )
         if int(sel_num) == 1:
             print("抓取的对象：试管")
             start_catch_position += [130.0]
             start_catch_position += rxryrz
             break
         elif int(sel_num) == 2:
             print("抓取的对象：烧杯")
             start_catch_position += [65.0]
             start_catch_position += rxryrz
             break
         elif int(sel_num) == 3:
             print("抓取的对象：量筒")
             start_catch_position += [65.0]
             start_catch_position += rxryrz
             break
         elif int(sel_num) == 4:
             print("抓取的对象：反应瓶")
             start_catch_position += [100.0]
             start_catch_position += rxryrz
             break
         else:
             print("--------输入错误！请重新输入！---------")
             continue
    #  self.Go_to_start_zone()
     if(int(sel_num)==4):
             self.MoveGripper(1, 50, 50, 10, 10000, 1)  # close
             time.sleep(3)
     print('==========================',start_catch_position)
    #  self.Safe_move(start_catch_position, dir)  # for display
     if catch_direction == "yn":
        # y方向留100距离 z方向留200距离
        position1 = start_catch_position
        position1[2] = position1[2] + 200
        self.robot.MoveCart(position1,0,0)
        time.sleep(2)
        self.MoveL(0.0,0.0,-200.0)
        time.sleep(1)
     elif catch_direction == "xn":
         # x方向留100距离 z方向留200距离
        position1 = start_catch_position
        position1[2] = position1[2] + 200
        self.robot.MoveCart(position1,0,0)
        time.sleep(2)
        self.MoveL(0.0,0.0,-200.0)
        time.sleep(1)
     elif catch_direction == "xp":
         # x方向留-100距离 z方向留200距离
        position1 = start_catch_position
        position1[2] = position1[2] + 200
        self.robot.MoveCart(position1,0,0)
        time.sleep(2)
        self.MoveL(0.0,0.0,-200.0)
        time.sleep(1)

     time.sleep(1)
     # 拿起+放下
     
     if(int(sel_num)==4):
        self.MoveGripper(1, 50, 50, 10, 10000, 1)  # Open
        time.sleep(3)
     else:
        self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
        time.sleep(3)
        
     print("动作完成")

    def Auto_Weight(self,fr5_B):
        '''
        自动称量
        '''
        ################ step1: 从仪器区抓一只量筒放到称量区称量 ###############
        string = ' '.join(map(str, liangtong_xy_input))
        fr5_B.F101_catch_02(string, "xp" , "2" , False, is_display = False)
        fr5_B.MoveL(0.0, 0.0, 300.0)
        p3_auto_weight = fr5_B.robot.GetForwardKin(j3_auto_weight)
        while( type(p3_auto_weight) != tuple):
                p3_auto_weight = fr5_B.robot.GetForwardKin(j3_auto_weight)
                time.sleep(0.1)
        p3_auto_weight = p3_auto_weight[1]
        fr5_B.robot.MoveJ(j3_auto_weight,0,0,p3_auto_weight,20.0,0,100.0,eP1,-1.0,0,oP1)
        time.sleep(2)
        string = ' '.join(map(str, liangtong_xy_output))
        fr5_B.F101_put_01(string, "xn" , "2" , False, is_display = False)
        ################ step2: 从仪器区抓一只烧杯放到称量区称量 ###############
        string = ' '.join(map(str, shaobei_xy_input))
        fr5_B.F101_catch_02(string, "xp" , "2" , False, is_display = False)
        fr5_B.MoveL(0.0, 0.0, 300.0)
        p3_auto_weight = fr5_B.robot.GetForwardKin(j3_auto_weight)
        while( type(p3_auto_weight) != tuple):
                p3_auto_weight = fr5_B.robot.GetForwardKin(j3_auto_weight)
                time.sleep(0.1)
        p3_auto_weight = p3_auto_weight[1]
        fr5_B.robot.MoveJ(j3_auto_weight,0,0,p3_auto_weight,20.0,0,100.0,eP1,-1.0,0,oP1)
        time.sleep(2)
        string = ' '.join(map(str, shaobei_xy_output))
        fr5_B.F101_put_01(string, "xn" , "2" , False, is_display = False)



        ################ step3：从仪器区抓起三颈烧瓶放到水浴锅夹持 ##############




        ################ step4：将量筒中液体倒入水浴锅 ########################




        ################ step5：将烧杯中固体倒入水浴锅 ########################




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
    
