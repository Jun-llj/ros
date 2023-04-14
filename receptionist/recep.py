#!/usr/bin/env python3
# coding: UTF-8
# Created by jun
# date: 2023/4/11

import rospy
from navigator import Navigator  # 导航模块
from soundplayer import Soundplayer  # 语音合成模块
from recep_voice_recognizer import Recognizer  # 语音识别模块和分析模块
# from pdfmaker import Pdfmaker  # pdf制作模块
from base_controller import Base  # 底盘运动模块
from std_msgs.msg import String  # std_msgs中包含消息类型string，发布的消息类型为String，从String.data中可获得信息，
from face import Face

LOCATION = {  # 储存导航路径点
    'reception point': [[-4.352973, -6.186659, 0.000000], [0.000000, 0.000000, -0.202218, -0.979341]],
    # 'detect point in living room': [[-0.476640, -4.946882, 0.000000], [0.000000, 0.000000, 0.808888, 0.587962]],
    'host point1': [[-1.658400, -0.046712, 0.000000], [0.000000, 0.000000, -0.986665, 0.162761]],
    'host point2': [[3.859466, -2.201285, 0.000000], [0.000000, 0.000000, -0.247601, -0.968862]],
    'guest1 point1': [[3.583689, 0.334696, 0.000000], [0.000000, 0.000000, -0.820933, -0.571025]],
    'guest1 point2': [[0.166213, 3.886673, 0.000000], [0.000000, 0.000000, -0.982742, 0.184983]],
    'guest2 point': [[-3.026017, -5.607293, 0.000000], [0.000000, 0.000000, -0.569564, 0.821947]],
}

class Controller:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)  # 初始化ros节点，告诉rospy你的节点名字，叫name，
        rospy.Subscriber('/start_signal', String, self.control)  # 创建订阅者订阅recognizer发出的地点作为启动信号
        self.navigator = Navigator(LOCATION)  # 实例化导航模块
        self.soundplayer = Soundplayer()  # 实例化语音合成模块
        self.recognizer = Recognizer()  # 实例化语音识别和逻辑判断模块
        # self.pdfmaker = Pdfmaker()  # 实例化pdf导出模块
        self.base = Base()  # 实例化移动底盘模块
        self.soundplayer.say("Please give me the command.", 3)  # 语音合成模块调用play方法传入字符串即可播放
        self.recognizer.get_cmd()  # 获取一次语音命令
        self.face = Face()  #实例化人脸识别

    def control(self, place):
        """订阅start signal的回调函数,传入的place是String类型消息 .data可以获取传来的信息,即目标房间"""
        self.goal = place.data  # 存入目标房间名字
        self.navigator.goto(place.data)
        while True:
            guest1_id = self.face.real_time_detect()
            if guest1_id == 'guest1':
                print(guest1_id)
                break
        self.soundplayer.say('客人1名字, please follow me.')
        self.navigator.goto('host point1')  # 导航模块调用goto方法,传入去的地点名字符串即可导航区指定地点
        self.soundplayer.say('Hello, 主人名字, this is 客人1名字, who likes 客人1饮料.')
        self.navigator.goto('guest1 point1')
        self.soundplayer.say('This is 客人1名字\'s seat')
        self.navigator.goto('reception point')
        while True:
            guest2_id = self.face.real_time_detect()
            if guest2_id == 'guest2':
                print(guest2_id)
                break
        self.soundplayer.say('客人2名字, please follow me.')
        self.navigator.goto('host point2')
        self.soundplayer.say('Hello, 主人名字 and 客人1名字, this is 客人2名字, who likes 客人2饮料.')
        self.soundplayer.say('客人2名字, this is 客人1名字, who likes 客人1饮料.')
        self.navigator.goto('guest2 point')
        self.soundplayer.say('This is 客人2名字\'s seat')

if __name__ == '__main__':
    try:
        Controller('reception')  # 实例化Controller,参数为初始化ros节点使用到的名字
        rospy.spin()  # 保持监听订阅者订阅的话题，直到节点已经关闭
    except rospy.ROSInterruptException:
        pass
