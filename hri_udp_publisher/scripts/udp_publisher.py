#!/usr/bin/env python
import rospy
import socket
from multiprocessing import Process, Queue
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
        # self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket.bind((self.ip, self.port))
        rospy.loginfo(f"UDP socket bounded to {self.ip}:{self.port}")

        # Process initialization
        rospy.loginfo('Waiting for header...')
        self.queue = Queue()
        self.process = Process(target=self.parse_stream)
        self.process.start()

    def __del__(self):
        # close the socket and terminate the reading process
        rospy.loginfo(f"Close UDP socket")
        self.udp_socket.close()
        rospy.loginfo(f"Terminate process")
        self.process.terminate()
        rospy.sleep(2)

    def parse_line(self, line):
        if len(line) < 1:
            rospy.logwarn('Empty line?')
            return

        if not self.header:
            if line[0] == 'H':
                self.header = line[1:].split(self.sep)
                rospy.loginfo('Header found.')
                rospy.loginfo('Start streaming...')
        else:
            if line[0] == 'J':
                d = dict(zip(self.header, line[1:].split(self.sep)))
                # remove the empty field from the tab at the end of the line if necessary
                if '' in d.keys():
                    del d['']
                self.queue.put(d)

    def parse_stream(self):
        udp_buffer_size = 4096
        stream_buffer = ''

        # Stream parsing
        while True:
            # accumulate data
            data, _ = self.udp_socket.recvfrom(udp_buffer_size)
            stream_buffer += data.decode('utf-8')

            # now divide it in lines
            lines = stream_buffer.splitlines()

            # if the stream doesn't end with a new line, it means we have received just
            # part of the last data point: Put the content back on the buffer.
            last_line_is_complete = stream_buffer.endswith('\n')
            if not last_line_is_complete:
                stream_buffer = lines.pop()

            for line in lines:
                self.parse_line(line)

    def __iter__(self):
        return self

    def __next__(self):
        # dict.keys together dict.values possibly not threadsafe (dict may change in between calls)! Therefore:
        self.journal.keys, self.journal.values = zip(*self.queue.get().items())
        return self.journal


def main():
    rospy.init_node('udp_publisher')
    topic = 'hri_udp_publisher/gaze_journal'
    pub = rospy.Publisher(topic, Journal, queue_size=1)
    rospy.loginfo(f"Publishing on {topic}")

    udp_parser = UDPParser()

    # try:
    while not rospy.is_shutdown():
        journal = next(udp_parser)
        pub.publish(journal)
    # except rospy.ROSInterruptException:
    #     del udp_parser


if __name__ == '__main__':
    main()
