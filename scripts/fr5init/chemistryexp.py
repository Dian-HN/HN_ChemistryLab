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
            2024.05.08,pour related functions are added, hcj
            2024.05.12，写了滴加KMnO4的大部分函数体，缺少蠕动泵电控和夹软管的操作，zjh
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
liangtong_xy_input = [600, 200] # 用于称量液体浓盐酸的量筒，对于B
shaobei_xy_input = [600, 100] # 用于称量固体C的烧杯，对于B
sanjing_xy_input = [520, -300] # 用于反应容器的三颈烧瓶，对于B
liangtong_xy_output = [-500, -200]
shaobei_xy_output = [-500, -300]
sanjing_xy_output = [220, -560, 220]
liangtong_xy_used = [600, 0]
shaobei_xy_used = [600, -100]
yindaoqi_xy = [570,-300]
liangtong_xy_kmno4_givetoB = [-200, -700]# 装有kmno4的量筒，需要递交给A，需要放在这个地方等待递交
liangtong_xy_kmno4_puttopump = [-600, -300]

'''
关键姿势的J与P，做伺服到位用
'''
j1_auto_weight = [-36.870, -59.151, -137.756, -163.093, -126.870, 0.000]
j2_auto_weight = [40.542, -57.601, -138.394, -164.005, -49.458, 0.000]
j3_auto_weight = [-64.941, -59.151, -137.756, -163.094, 25.056,0.001]
j4_auto_weight = [-49.458, -55.125, -124.682, -180.193, -49.458, 0.000]# 这个是机械臂复位的J
j5_auto_weight = [-119.608, -101.188, -110.403, -148.409, -119.608, 0.000]# 这是机械臂A要靠过来抓三颈烧瓶时的先验关节角度
j7_auto_weight = [-27.158, -90.814, -93.062, -174.026, -27.173, 0.384]# 这是机械臂B抓取到引导器后，准备放到水浴锅上方之前的过渡动作
j8_auto_weight = [-84.385, -71.425, -117.697, -170.878, 5.615, 0.000]

p6_auto_weight = [320.0, -340.0, 400.0, 90.0, 0.0 ,0.0]




class HNchemistry(fr5robot):
    def __init__(self, index=1):
        super().__init__(index)
        self.robot.SetSpeed(60)

    def F101_catch_02(self,start_catch_position,catch_direction,sel_num,is_force_sensor = False,is_display = False):
     
     '''
    指定坐标抓取指定物体
    预期效果： 机械臂回到起始区————运动到指定位置抓取物体————抬起展示抓取效果————松开夹爪回到结束区
        start_catch_position: 目标物体绝对xy坐标
        catch_direction: ym——y轴负方向  xm——x负方向
        sel_num:抓取的对: 1---试管 2---烧杯 3---量筒 4---反应瓶 5---引导器
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
             start_catch_position += [155.0]
             start_catch_position += rxryrz
             break
         elif int(sel_num) == 5:
             print("抓取的对象：引导器")
             start_catch_position += [130.0]
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
        self.MoveGripper(1, 0, 50, 30, 10000, 1)  # close
        time.sleep(3)
     else:
        self.MoveGripper(1, 0, 50, 30, 10000, 1)  # close
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
        print('==========================wancheng',start_catch_position)
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
     
    def F101_pour_01(self,pour_position,pour_direction,sel_num,clockwise = False):
        '''
        指定位置倾倒指定仪器
        预期效果： 机械臂运动到指定位置————机械臂倾倒仪器————机械臂等待倾倒完毕————机械臂抖动————机械臂回到一个中立位置
        pour_position: 目标绝对xyz坐标
        catch_direction: ym——y轴负方向  xm——x负方向
        sel_num: 1---试管 2---烧杯 3---量筒 4---反应瓶
        is_force_sensor:末端是否有力传感器
        is_display:是否为演示，如果是，机械臂会自动复位
        '''
        print("目标物体绝对xyz坐标:",pour_position)
        pour_position = pour_position.split(" ")
        pour_position = [float(num) for num in pour_position]
        rxryrz = []

        if pour_direction == "yn":
            print("倾倒方向是：y轴负方向")
            # 数据处理y
            pour_position[1] = pour_position[1] + 150.0
            pour_position[0] = pour_position[0]
            if clockwise:
                rxryrz = [90.0, -30.0, 0.0]
            else:
                rxryrz = [90.0, 30.0, 0.0]
            dir = "yn"
        elif pour_direction == "xn":
            print("倾倒方向是：x轴负方向")
            # 数据处理x
            pour_position[0] = pour_position[0] + 150.0
            rxryrz = [90.0, 0.0, -90.0]
            dir = "xn"
        elif pour_direction == "xp":
            print("倾倒方向是：x轴正方向")
            # 数据处理x
            pour_position[0] = pour_position[0] - 150
            rxryrz = [90.0, 0.0, 90.0]
            dir = "xp"
        else:
         print("------error!-------")
         exit()
        # 合法检测
        if len(pour_position) != 3:
            print("xyz坐标错误")
            exit()
        else:
            print("----------------------")   
        
        if int(sel_num) == 1:
            print("倾倒的对象：试管")
            pour_position[2] += 10.0
            bias = 20
            height = 10
        elif int(sel_num) == 2:
            print("倾倒的对象：烧杯")
            pour_position[2] += 10.0
            bias = 37
            height = 35
        elif int(sel_num) == 3:
            print("倾倒的对象：量筒")
            pour_position[2] += 10.0
            bias = 20
            height = 50     
        else:
            print("--------Wrong sel index!---------")
            exit()

        pour_position += rxryrz
        print('==========================',pour_position)


        if pour_direction == "yn":
            # y方向留bias距离 z方向留height距离
            position1 = pour_position
            if clockwise:
                position1[0] = position1[0] - bias
            else:
                position1[0] = position1[0] + bias
            position1[2] = position1[2] + height * np.cos(np.abs(rxryrz[1]) * np.pi / 180)
            self.robot.MoveCart(position1,0,0)
            time.sleep(2)
        elif pour_direction == "xn":
            # x方向留bias距离 z方向留height距离
            position1 = pour_position
            position1[0] = position1[0] + bias
            position1[2] = position1[2] + height
            self.robot.MoveCart(position1,0,0)
            time.sleep(2)
        elif pour_direction == "xp":
            # x方向留bias距离 z方向留height距离
            position1 = pour_position
            position1[0] = position1[0] - bias
            position1[2] = position1[2] + height
            self.robot.MoveCart(position1,0,0)
            time.sleep(2)
        max_angle = 100
        pre_angle = np.abs(90 - np.abs(rxryrz[1]))

        if clockwise == False:
            bias = -bias
            
        self.pour(np.abs(bias), height, float(2 * np.sign(bias)), max_angle - np.abs(pre_angle), 100)
        time.sleep(1)
        self.Go_to_start_zone(open=False)
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
        fr5_B.MoveL(0.0, 0.0, 300.0)
        fr5_B.Go_to_start_zone()


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
        fr5_B.MoveL(0.0, 0.0, 300.0)
        fr5_B.Go_to_start_zone()


        ################ step3：从仪器区抓起三颈烧瓶放到水浴锅夹持 ##############
        string = ' '.join(map(str, sanjing_xy_input))
        # 三颈烧瓶抓取方式需要修改与测量 TODO
        # fr5_B.F101_catch_02(string, "xp" , "4" , False, is_display = False)
        fr5_B.F101_catch_02(string, "xp" , "4" , False, is_display = False)
        fr5_B.MoveL(0.0, 0.0, 170.0)
        time.sleep(2)
        fr5_B.MoveL(-370.0, 0.0, 0.0)
        p4_auto_weight = fr5_B.robot.GetForwardKin(j4_auto_weight)
        while( type(p4_auto_weight) != tuple):
                p4_auto_weight = fr5_B.robot.GetForwardKin(j4_auto_weight)
                time.sleep(0.1)
        p4_auto_weight = p4_auto_weight[1]
        fr5_B.robot.MoveJ(j4_auto_weight,0,0,p4_auto_weight,20.0,0,100.0,eP1,-1.0,0,oP1)
        time.sleep(2)
        fr5_B.robot.MoveCart(p6_auto_weight,0,0)
        time.sleep(2)
        fr5_B.MoveL(0.0,0.0,-145.0)
        # 启动另一个机械臂a，过来抓住，并且放好位置
        
        p5_auto_weight = self.robot.GetForwardKin(j5_auto_weight)
        while( type(p5_auto_weight) != tuple):
                p5_auto_weight = self.robot.GetForwardKin(j5_auto_weight)
                time.sleep(0.1)
        p5_auto_weight = p5_auto_weight[1]
        self.robot.MoveJ(j5_auto_weight,0,0,p5_auto_weight,20.0,0,100.0,eP1,-1.0,0,oP1)
        time.sleep(2)
        # input('回车，a会夹住')
        self.MoveL(0.0,-50.0,0.0)
        time.sleep(1)
        self.robot.MoveGripper(1, 0, 50, 30, 10000, 1)
        time.sleep(2)
        
        # 机械臂B继续工作：
        
        fr5_B.robot.MoveGripper(1, 100, 50, 10, 10000, 1)
        time.sleep(2)
        fr5_B.MoveL(0.0,50.0,0.0)
        time.sleep(1)
        fr5_B.MoveL(0.0,0.0,100.0)
        time.sleep(1)
        fr5_B.Go_to_start_zone()



        ################ step4：将量筒中液体倒入水浴锅 ########################
        # 机械臂B抓取称量后的量筒
        string = ' '.join(map(str, liangtong_xy_output))
        fr5_B.F101_catch_02(string, "xn" , "3" , False, is_display = False)
        fr5_B.MoveL(0.0, 0.0, 150.0)
        
        # 机械臂移动到水浴锅中三颈烧瓶上方，执行倾倒
        string = ' '.join(map(str, sanjing_xy_output))
        fr5_B.F101_pour_01(string, "yn" , "3")
        
        # 机械臂B将倾倒后的量筒放置在指定位置
        string = ' '.join(map(str, liangtong_xy_used))
        fr5_B.F101_put_01(string, "xp" , "3" , False, is_display = False)
        fr5_B.Go_to_start_zone()

        ################ step5：将烧杯中固体倒入水浴锅 ########################
        # 机械臂B抓取称量后的烧杯
        string = ' '.join(map(str, shaobei_xy_output))
        fr5_B.F101_catch_02(string, "xn" , "2" , False, is_display = False)
        fr5_B.MoveL(0.0, 0.0, 150.0)
        
        # 机械臂移动到水浴锅中三颈烧瓶上方，执行倾倒
        string = ' '.join(map(str, sanjing_xy_output))
        fr5_B.F101_pour_01(string, "yn" , "2")
        
        # 机械臂B将倾倒后的量筒放置在指定位置
        string = ' '.join(map(str, shaobei_xy_used))
        fr5_B.F101_put_01(string, "xp" , "2" , False, is_display = False)
        fr5_B.Go_to_start_zone()


    def Add_Solid_Liquid(self):
        '''
        固液加料实验
        '''
        
        ################ step1：将量筒中液体倒入水浴锅 ########################




        ################ step2：将烧杯中固体倒入水浴锅 ########################


        pass

    def Mix_Solid_Liquid(self,fr5_B):
        '''
        固液混合实验
        '''
        pass

    def Add_KMnO4(self,fr5_B):
        '''
        添加KMnO4
        '''
        #########################step1:catch the cylinder to weight######################
        string = ' '.join(map(str, liangtong_xy_input))
        fr5_B.F101_catch_02(string, "xp" , "2" , False, is_display = False)
        fr5_B.MoveL(0.0, 0.0, 300.0)
        # 去复位的位置
        p4_auto_weight = fr5_B.robot.GetForwardKin(j4_auto_weight)
        while( type(p4_auto_weight) != tuple):
                p4_auto_weight = fr5_B.robot.GetForwardKin(j4_auto_weight)
                time.sleep(0.1)
        p4_auto_weight = p4_auto_weight[1]
        fr5_B.robot.MoveJ(j4_auto_weight,0,0,p4_auto_weight,20.0,0,100.0,eP1,-1.0,0,oP1)
        time.sleep(2)
        
        #########################step2:put cylinder to a place######################
        string = ' '.join(map(str, liangtong_xy_kmno4_givetoB))
        fr5_B.F101_put_01(string, "yn" , "2" , False, is_display = False)
        fr5_B.MoveL(0.0, 0.0, 100.0)
        fr5_B.Go_to_start_zone()
        
        #########################step3:armA catch the cylinder to pump######################
        self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
        self.Go_to_start_zone()
        string = ' '.join(map(str, liangtong_xy_output))
        self.F101_catch_02(string, "yn" , "2" , False, is_display = False)
        self.MoveL(0.0, 0.0, 300.0)
        # 去A准备放下量筒的位置
        p8_auto_weight = self.robot.GetForwardKin(j8_auto_weight)
        while( type(p8_auto_weight) != tuple):
                p8_auto_weight = self.robot.GetForwardKin(j8_auto_weight)
                time.sleep(0.1)
        p8_auto_weight = p8_auto_weight[1]
        self.robot.MoveJ(j8_auto_weight,0,0,p8_auto_weight,20.0,0,100.0,eP1,-1.0,0,oP1)
        time.sleep(2)
        # 把量筒放下
        string = ' '.join(map(str, liangtong_xy_kmno4_puttopump))
        self.F101_put_01(string, "xn" , "2" , False, is_display = False)
        self.MoveL(0.0,0.0,300.0)
        self.Go_to_start_zone()
        
        
        #########################step4: armA catch the soft pump######################



        #########################step5:b catch the guide model###########################
        string = ' '.join(map(str, yindaoqi_xy))
        fr5_B.F101_catch_02(string, "xp" , "5" , False, is_display = False)
        fr5_B.MoveL(0.0, 0.0, 300.0)
        p7_auto_weight = fr5_B.robot.GetForwardKin(j7_auto_weight)
        while( type(p7_auto_weight) != tuple):
                p7_auto_weight = fr5_B.robot.GetForwardKin(j7_auto_weight)
                time.sleep(0.1)
        p7_auto_weight = p7_auto_weight[1]
        fr5_B.robot.MoveJ(j7_auto_weight,0,0,p7_auto_weight,20.0,0,100.0,eP1,-1.0,0,oP1)
        time.sleep(2)
        # 引导器降落到三颈烧瓶上方
        fr5_B.MoveL(0.0,0.0,-100.0)
        
        
        
        #########################step6:start the pump###########################
        #TODO




        #########################step7: wait for pump ###########################



        #########################step8: reset   ###########################




        pass

    def Add_H2O2(self):
        '''
        添加H2O2
        '''
        #########################step1:   ###########################



        #########################step2:   ###########################

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
    
