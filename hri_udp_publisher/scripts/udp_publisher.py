#!/usr/bin/env python
import rospy
import socket
from hri_udp_publisher.msg import Journal


class UDPSocket:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("localhost", 2002))

    def read_journal(self):
        data, _ = self.sock.recvfrom(512)  # buffer size is 512
        data = data.decode('UTF-8')
        data = data.split("\t")
        data = data[:54]
        return data


def main():
    rospy.init_node('udp_publisher')
    pub = rospy.Publisher('hri_udp_publisher/gaze_journal', Journal, queue_size=1)

    udp_socket = UDPSocket()

    while not rospy.is_shutdown():
        journal = udp_socket.read_journal()
        pub.publish(journal)


if __name__ == '__main__':
    main()
