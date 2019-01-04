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


#ifndef INCLUDED_ROS_INTERFACE_RATE_DEBUG_H
#define INCLUDED_ROS_INTERFACE_RATE_DEBUG_H

#include <ros_interface/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace ros_interface {

    /*!
     * \brief <+description of block+>
     * \ingroup ros_interface
     *
     */
    class ROS_INTERFACE_API rate_debug : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<rate_debug> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of ros_interface::rate_debug.
       *
       * To avoid accidental use of raw pointers, ros_interface::rate_debug's
       * constructor is in a private implementation
       * class. ros_interface::rate_debug::make is the public interface for
       * creating new instances.
       */
      static sptr make();
    };

  } // namespace ros_interface
} // namespace gr

#endif /* INCLUDED_ROS_INTERFACE_RATE_DEBUG_H */

