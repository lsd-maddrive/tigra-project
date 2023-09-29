#!/usr/bin/env python
import rospy
from mavros_msgs.msg import RTCM
from rtcm_msgs.msg import Message as RTCMMessage
from std_msgs.msg import Header  # Добавлен импорт для std_msgs/Header

def mavros_to_rtcm_callback(mavros_msg):
    rtcm_msg = RTCMMessage()
    
    # Копирование данных из mavros_msg в rtcm_msg
    rtcm_msg.header = mavros_msg.header  # Копирование заголовка
    rtcm_msg.message = mavros_msg.data  # Копирование данных

    # Публикация преобразованного сообщения на новой теме
    rtcm_pub.publish(rtcm_msg)

if __name__ == '__main__':
    rospy.init_node('mavros_to_rtcm_converter')

    # Создание подписчика на тему с mavros_msgs/RTCM
    mavros_sub = rospy.Subscriber('/ntrip_client/rtcm', RTCM, mavros_to_rtcm_callback)

    # Создание издателя на тему с rtcm_msgs/Message
    rtcm_pub = rospy.Publisher('/rtcm', RTCMMessage, queue_size=10)

    rospy.spin()  # Запуск бесконечного цикла для обработки сообщений