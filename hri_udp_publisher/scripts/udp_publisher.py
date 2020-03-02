#!/usr/bin/env python
import select
import rospy
import socket
from hri_udp_publisher.msg import Journal


class UDPParser:
    """Class to receive and parse EyeRec data as a Python iterable

        By default EyeRec broadcasts through UDP on port 2002.
        This class reads the data stream until a header package has been found.
        After a header has been found, it starts producing and publishing dicts
        containing the journal data, where the keys are given by the header.
    """

    def __init__(self, ip="localhost", port=2002):
        self.ip = ip
        self.port = port
        self.header = None
        self.sep = '\t'
        self.journal = Journal()

        # UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((self.ip, self.port))
        rospy.loginfo(f"UDP socket bound to {self.ip}:{self.port}")
        rospy.loginfo('Waiting for header...')

    def __del__(self):
        # close the socket
        rospy.loginfo(f"Stop parsing")
        self.udp_socket.close()
        rospy.loginfo(f"UDP socket closed")

    def parse_line(self, line):
        if len(line) < 1:
            rospy.logwarn('Empty line?')
            return

        if not self.header:
            if line[0] == 'H':
                self.header = line[1:].split(self.sep)
                rospy.loginfo('Header found')
                rospy.loginfo('Start streaming...')
        else:
            if line[0] == 'J':
                d = dict(zip(self.header, line[1:].split(self.sep)))
                # remove the empty field from the tab at the end of the line if necessary
                if '' in d.keys():
                    del d['']
                self.journal.header.stamp = rospy.Time.now()
                self.journal.keys = d.keys()
                self.journal.values = d.values()
                # dict.keys together dict.values possibly not threadsafe (dict may change in between calls).
                # self.journal.keys, self.journal.values = zip(*self.queue.get().items())

    def parse_stream(self):
        udp_buffer_size = 1024

        # Check for half a second whether UDP data is received
        ready, _, _ = select.select([self.udp_socket], [], [], 0.5)

        # Return if no data is received
        if self.udp_socket not in ready:
            if self.header is not None:
                rospy.logwarn('No data received')
                rospy.loginfo('Stop streaming')
                rospy.loginfo('Waiting for header...')
                self.header = None
            return

        # read data
        data_stream, _ = self.udp_socket.recvfrom(udp_buffer_size)
        data_stream = data_stream.decode('utf-8')

        # divide it in lines
        lines = data_stream.splitlines()

        # if the stream doesn't end with a new line, then last line is not complete
        last_line_is_complete = data_stream.endswith('\n')

        # delete incomplete line
        if not last_line_is_complete:
            lines.pop()

        # Parse last (most recent) line
        self.parse_line(lines[-1])

    def __iter__(self):
        return self

    def __next__(self):
        self.parse_stream()
        if self.header is None:
            return False, self.journal
        else:
            return True, self.journal


def main():
    rospy.init_node('udp_publisher')
    topic = 'hri_udp_publisher/gaze_journal'
    pub = rospy.Publisher(topic, Journal, queue_size=1)
    rospy.loginfo(f"Publishing to {topic}")

    udp_parser = UDPParser()

    while not rospy.is_shutdown():
        ret, journal = next(udp_parser)
        if ret is True:
            pub.publish(journal)


if __name__ == '__main__':
    main()
