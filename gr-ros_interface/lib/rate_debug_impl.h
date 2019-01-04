/* -*- c++ -*- */
/* 
 * Copyright 2019 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_ROS_INTERFACE_RATE_DEBUG_IMPL_H
#define INCLUDED_ROS_INTERFACE_RATE_DEBUG_IMPL_H

#include <ros_interface/rate_debug.h>
#include <chrono>  // for high_resolution_clock

namespace gr {
  namespace ros_interface {

    class rate_debug_impl : public rate_debug
    {
     private:
    	unsigned long int d_rcv_msgs;
    	unsigned long int d_rcv_bytes;
    	std::chrono::high_resolution_clock::time_point d_begin;

     public:
    	rate_debug_impl();
    	~rate_debug_impl();
    	void update_rate(pmt::pmt_t msg);
    };

  } // namespace ros_interface
} // namespace gr

#endif /* INCLUDED_ROS_INTERFACE_RATE_DEBUG_IMPL_H */

