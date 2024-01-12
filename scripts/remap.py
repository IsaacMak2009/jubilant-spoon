import importlib
import sys
import rospy
import argparse
import time
from typing import *

class ReMapper:
    def __init__(self, topic1: str, topic2: str, type_str: str):
        self.topic1 = topic1
        self.topic2 = topic2
        self.frame = 0
        self.start_time = time.time()
        self.last_time = time.time()
        self.tot_size = 0
        self.size = 0
        self.hz = 0
        self._type_root, self._type_str = type_str.split("/")
        self._type_root = self._type_root.lower() + ".msg"
        self._type = getattr(importlib.import_module(self._type_root), self._type_str)
        self.sub = rospy.Subscriber(topic1, self._type, self.callback)
        self.pub = rospy.Publisher(topic2, self._type, queue_size=10)
        for i in range(7):
            print()
        while not rospy.is_shutdown():
            print("\033[1G\033[7A")
            print()
            print(f"\033[1mTopic type: \033[0m\033[34m{type_str} ({self._type})\033[0m")
            print(f"\033[1mTotal frames: \033[0m\033[34m{self.frame} \033[0m({self.tot_size/8/1024:.5f}kb @ {self.size/8} bytes per frame)")
            print(f"\033[1mFreq: \033[0m\033[34m{self.hz:.2f} hz \033[0m(ping: {int((time.time() - self.last_time)*1000)}ms)          ")
            print()
            print(f" * \033[1mRemapping: \033[0m\033[34m{self.topic1} {update_bar((self.frame//10)%20)} {self.topic2}\033[0m")
            time.sleep(0.075)
        
    def callback(self, data):
        self.hz = 1 / (time.time() - self.last_time)
        self.last_time = time.time()
        self.pub.publish(data)
        self.tot_size += sys.getsizeof(data)
        self.size = sys.getsizeof(data)
        self.frame += 1
            
        
        
def update_bar(progress):
    bar_length = 20
    filled_length = progress
    remaining_length = bar_length - filled_length
    bar = '[' + ' ' * filled_length + '>' + ' ' * remaining_length + ']'
    return bar


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('topic1', type=str, help='The first ROS topic. (source)')
    parser.add_argument('topic2', type=str, help='The second ROS topic. (target)')
    parser.add_argument('--type', type=str, help='Specify the type of topic')

    args = parser.parse_args()
    rospy.init_node("remapper", anonymous=True)
    ReMapper(args.topic1, args.topic2, args.type)
