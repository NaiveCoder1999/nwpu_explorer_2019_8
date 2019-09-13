#!/usr/bin/env python3
#coding=utf-8

import rospy
from explorer_msgs.msg import explorer_low_level_data
from explorer_msgs.msg import explorer_message
from std_msgs.msg import String
from threading import Thread, Lock
#import numpy as np
import tkinter as tk

data = 'Co2 : 0 ppm'
mutex = Lock()
co2_req_pub_ = rospy.Publisher('/explorer_driver',
                               explorer_message,
                               queue_size=10)


def sendCo2Request(publisher):
    co2_req_msg = explorer_message()
    co2_req_msg.high_level_id = 9
    co2_req_msg.low_level_id = 9
    co2_req_msg.data.append(0)
    co2_req_msg.data.append(1.0)
    publisher.publish(co2_req_msg)


def start():
    for x in range(100):
        sendCo2Request(co2_req_pub_)
        x += 1


#subscriber仅用于更新data数值
def co2Callback(msg):
    global data
    mutex.acquire()
    data = 'Co2 : ' + str(msg.can_serial_data_1) + 'ppm'
    mutex.release()
    print(data)


def co2CB():
    global data, mutex
    #以下函数为窗口更新的关键
    l.configure(text=data)
    window.after(500, co2CB)


if __name__ == '__main__':
    try:
        rospy.init_node('explorer_co2_plot', anonymous=False)
        rate = rospy.Rate(10)
        co2_sub_ = rospy.Subscriber('/explorer_serial_data/9',
                                    explorer_low_level_data, co2Callback)
        #初始化窗口
        window = tk.Tk()
        window.title('Co2 Detection')
        window.geometry('640x480')
        btnStart = tk.Button(window,
                             text="start",
                             font=('Arial', 20),
                             width=15,
                             height=2,
                             command=start)
        btnStart.pack(side='bottom')
        l = tk.Label(window,
                     text=data,
                     font=('Arial', 50),
                     width=20,
                     height=20)
        l.pack()
        l.after(500, co2CB)
        window.mainloop()
        #此线程用于subscriber
        t = Thread(target=rospy.spin)
        t.setDaemon(True)
        t.start()
    except rospy.ROSInterruptException:
        pass

'''
Refernces:
https://blog.csdn.net/ztchun/article/details/69524485
https://morvanzhou.github.io/tutorials/python-basic/tkinter/2-01-label-button/
https://answers.ros.org/question/106781/rospy-and-tkinter-spin-and-mainloop/
'''
