#!/usr/bin/env python
import rospy
import socket
from hri_udp_publisher.msg import Journal


class UDPSocket:
    def __init__(self):
        address = "localhost"
        port = 2002
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((address, port))
        rospy.loginfo(f"UDP socket bounded to {address}:{port}")
        self.journal = Journal()

    def read_journal(self):
        data, _ = self.sock.recvfrom(512)  # buffer size is 512
        data = data.decode('UTF-8')
        data = data.split("\t")
        self.journal.data = data[:54]
        return self.journal


def main():
    udp_socket = UDPSocket()

    rospy.init_node('udp_publisher')
    topic = 'hri_udp_publisher/gaze_journal'
    pub = rospy.Publisher(topic, Journal, queue_size=1)
    rospy.loginfo(f"Publishing on {topic}")

    while not rospy.is_shutdown():
        journal = udp_socket.read_journal()
        pub.publish(journal)


if __name__ == '__main__':
    main()
