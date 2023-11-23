#!/usr/bin/python3

import rospy
import socket
import struct
from geometry_msgs.msg import PoseStamped
import threading

def receive_udp_message(multicast_group, multicast_port):
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Bind the socket to the multicast group and port
    sock.bind(('', multicast_port))

    # Join the multicast group
    group = socket.inet_aton(multicast_group)
    mreq = struct.pack('4sL', group, socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    # Create a publisher for the received messages
    received_msg_pub = rospy.Publisher('/pose_received', PoseStamped, queue_size=10)
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        # Receive data from the socket
        data, addr = sock.recvfrom(1024)

        # Deserialize the received message
        msg = PoseStamped()
        msg.deserialize(data)

        # Get the IP address of the transmitter
        transmitter_ip = addr[0]

        # Publish the received message with the transmitter's IP
        #rospy.loginfo("Received message from %s" % transmitter_ip)
        received_msg_pub.publish(msg)
        rate.sleep()
    
def main():
    # Initialize the ROS node
    rospy.init_node('udp_receiver_node', anonymous=True)
    rate = rospy.Rate(2)

    # Set the multicast group and port
    multicast_group = '224.0.0.251'
    multicast_port = 1901

    # Receive UDP messages in a separate thread
    rospy.loginfo("Listening for UDP messages on %s:%d" % (multicast_group, multicast_port))
    receive_thread = threading.Thread(target=receive_udp_message, args=(multicast_group, multicast_port))
    receive_thread.start()

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass