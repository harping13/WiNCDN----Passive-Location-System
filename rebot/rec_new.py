#!/usr/bin/env python2
# coding=utf8

import set_move_new
import json
import socket
import rospy

class UDPio:
    @staticmethod
    def GetHostIP():
        try:
            Socke = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Socke.connect(('192.168.1.112', 8080))
            Socke.connect(('192.168.1.121', 9090))
            IP = Socke.getsockname()[0]
        finally:
            Socke.close()
        return str(IP)

    @staticmethod
    def ReceiveUdpMessage():
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        IP_num = UDPio.GetHostIP()
        dest_addr = (IP_num, 4848)
        udp_socket.bind(dest_addr)
        print(udp_socket)
        move_and_capture = set_move_new.MoveAndCapture()
        while not rospy.is_shutdown():
            recv_data = udp_socket.recvfrom(4848)
            result_recv_data = recv_data[0].decode('gbk')
            print("Receive data: " + str(result_recv_data))
            # Move
            goal_list = json.loads(str(result_recv_data))['goallist']
            for goal in goal_list:
                move_and_capture.send_goal(goal[0], goal[1])
                while not move_and_capture.goal_reached and not rospy.is_shutdown():
                    rospy.sleep(1)  # 等待直到到达目标点

if __name__ == '__main__':
    try:
        UDPio.ReceiveUdpMessage()
    except rospy.ROSInterruptException:
        rospy.loginfo("UDP receiver node terminated.")

