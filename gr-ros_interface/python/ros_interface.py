#!/usr/bin/env python
# 
# Copyright 2018 IBM
# 
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

import numpy
import pmt
import rospy
import pickle
import bz2
import sys
from gnuradio import gr
#from nav_msgs.msg import OccupancyGrid
from era_gazebo.msg import ERAOccupancyGrid

class ros_interface(gr.basic_block):
    """
    docstring for block ros_interface
    """
    def __init__(self, ros_namespace, compress_topic):
        self.ros_namespace=ros_namespace
        self.compress_topic=compress_topic
        gr.basic_block.__init__(self,
            name="ros_interface",
            in_sig=None,
            out_sig=None)
        
        self.sent_msgs = 0
        self.received_msgs = 0

        self.subscribed_ros_topic = "local_map" 
        self.published_ros_topic = "remote_map"
        
        # Create input and output message ports
        self.message_port_register_in(pmt.intern("pdus"))
        self.message_port_register_out(pmt.intern("pdus"))
        self.set_msg_handler(pmt.intern('pdus'), self.handle_msg)
        rospy.init_node('ros_interface')
        rospy.Subscriber(self.subscribed_ros_topic, ERAOccupancyGrid, self.handle_ros_topic)
        self.pub = rospy.Publisher(self.published_ros_topic, ERAOccupancyGrid, queue_size=10)

    def general_work(self, input_items, output_items):
        rospy.spin()
    
    def handle_ros_topic(self, data):
        payload = pickle.dumps(data)
        rospy.loginfo("[%s][%d] Compressing %d bytes", rospy.get_caller_id(), self.sent_msgs, len(payload))
        compressed_payload = bz2.compress(payload)
        rospy.loginfo("[%s][%d] Sending %d bytes", rospy.get_caller_id(), self.sent_msgs, len(compressed_payload))
        self.message_port_pub(pmt.intern('pdus'), pmt.intern(compressed_payload))
        self.sent_msgs += 1

    def handle_msg(self, msg):
        compressed_payload = "".join([chr(x) for x in pmt.u8vector_elements(pmt.cdr(msg))])
        rospy.loginfo("[%s][%d] Receiving %d bytes", rospy.get_caller_id(), self.received_msgs, len(compressed_payload))
        payload = bz2.decompress(compressed_payload)
        rospy.loginfo("[%s][%d] Uncompressed size: %d bytes", rospy.get_caller_id(), self.received_msgs, len(payload))
        data = pickle.loads(payload)
        self.pub.publish(data)
        self.received_msgs += 1
