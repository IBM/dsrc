/*
 * Copyright 2018 IBM
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INCLUDED_ROS_INTERFACE_ROS_INTERFACE_IMPL_H
#define INCLUDED_ROS_INTERFACE_ROS_INTERFACE_IMPL_H

#include <ros_interface/ros_interface.h>
#include <gnuradio/blocks/pdu.h>
#include "ros/ros.h"
#include "ERAMsg.h"   // TODO: fix this header inclusion

namespace gr {
  namespace ros_interface {

    class ros_interface_impl : public ros_interface
    {
     private:
       unsigned int _sent_msgs;
       unsigned int _received_msgs;
       ros::NodeHandle _n;
       ros::Subscriber _loc_map_sub;
       ros::Publisher _rem_map_pub;
       ros::AsyncSpinner _spinner;
       std::string ID;

     public:
      ros_interface_impl(const std::string& robot_name);
      ~ros_interface_impl();

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

      void handle_ros_topic(const era_gazebo::ERAMsg msg);
      void handle_msg(pmt::pmt_t pdu);
    };

  } // namespace ros_interface
} // namespace gr

#endif /* INCLUDED_ROS_INTERFACE_ROS_INTERFACE_IMPL_H */

