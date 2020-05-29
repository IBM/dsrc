/* -*- c++ -*- */

#define ROS_INTERFACE_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "ros_interface_swig_doc.i"

%{
#include "ros_interface/ros_interface.h"
#include "ros_interface/message_strobe_custom.h"
#include "ros_interface/rate_debug.h"
#include "ros_interface/udp_broadcast_sink.h"
#include "ros_interface/udp_broadcast_source.h"
%}


%include "ros_interface/ros_interface.h"
GR_SWIG_BLOCK_MAGIC2(ros_interface, ros_interface);
%include "ros_interface/message_strobe_custom.h"
GR_SWIG_BLOCK_MAGIC2(ros_interface, message_strobe_custom);
%include "ros_interface/rate_debug.h"
GR_SWIG_BLOCK_MAGIC2(ros_interface, rate_debug);
%include "ros_interface/udp_broadcast_sink.h"
GR_SWIG_BLOCK_MAGIC2(ros_interface, udp_broadcast_sink);
%include "ros_interface/udp_broadcast_source.h"
GR_SWIG_BLOCK_MAGIC2(ros_interface, udp_broadcast_source);