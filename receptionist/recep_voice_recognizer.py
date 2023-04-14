#!/usr/bin/env python
# coding: UTF-8 
# Created by Cmoon

import rospy
from std_msgs.msg import String
from soundplayer import Soundplayer
# from pdfmaker import Pdfmaker

LOCATION = {  # 模糊词
    'reception point': ['reception point', 'receiption point', 'leception point'],
    'host point1': ['host point1', 'hos point1', 'host poin1', 'hos poin1'],
    'host point2': ['host point2', 'hos point2', 'host poin2', 'hos poin2'],
    'guest1 point1': ['guest1 point1', 'gues1 point1', 'gues1 poin1'],
    'guest1 point2': ['guest1 point2', 'gues1 point2', 'gues1 poin2'],
    'guest2 point': ['guest2 point', 'gues point', 'gues poin'],
}

class Recognizer:
    def __init__(self):
        rospy.Subscriber('/xfspeech', String, self.talkback)
        self.wakeup = rospy.Publisher('/xfwakeup', String, queue_size=10)
        self.start_signal = rospy.Publisher('/start_signal', String, queue_size=10)
        self.cmd = None
        self.location = LOCATION
        self.goal = ''
        self._soundplayer = Soundplayer()
        # self._pdfmaker = Pdfmaker()
        self.status = 0
        self.key = 1

    def talkback(self, msg):
        if self.key == 1:
            print("\n讯飞读入的信息为: " + msg.data)
            self.cmd = self.processed_cmd(msg.data)
            self.judge()

    def judge(self):
        if self.status == 0:

            response = self.analyze()

            if response == 'Do you need me':
                self._soundplayer.say("Please say the command again. ")
                self.get_cmd()
            else:
                self.status = 1
                print(response)
                self._soundplayer.say(response, 3)
                self._soundplayer.say("please say yes or no.", 1)
                print('Please say yes or no.')
                self.get_cmd()

        elif ('Yes.' in self.cmd) or ('yes' in self.cmd) or ('Yeah' in self.cmd) or ('yeah' in self.cmd) and (
                self.status == 1):

            self._soundplayer.say('Ok, I will.')
            # self._pdfmaker.write('Cmd: Do you need me go to the ' + self.goal + '?')
            # self._pdfmaker.write('Respond: Ok,I will.')
            print('Ok, I will.')
            self.start_signal.publish(self.goal)
            self.key = 0
            self.status = 0
            self.goal = ''


        elif ('No.' in self.cmd) or ('no' in self.cmd) or ('oh' in self.cmd) or ('know' in self.cmd) and (
                self.status == 1):
            self._soundplayer.say("Please say the command again. ")
            print("Please say the command again. ")
            self.status = 0
            self.goal = ''
            self.get_cmd()

        else:
            self._soundplayer.say("please say yes or no.")
            print('Please say yes or no.')
            self.get_cmd()

    def processed_cmd(self, cmd):
        cmd = cmd.lower()
        for i in " ,.;?":
            cmd = cmd.replace(i, ' ')
        return cmd

    def get_cmd(self):
        """获取一次命令"""
        self._soundplayer.play('Speak.')
        self.wakeup.publish('ok')

    def analyze(self):
        response = 'Do you need me'
        for (key, val) in self.location.items():
            for word in val:
                if word in self.cmd:
                    self.goal = key
                    response = response + ' go to the ' + key + '?'
                    break
        return response


if __name__ == '__main__':
    try:
        rospy.init_node('voice_recognition')
        Recognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
