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

#ifndef INCLUDED_ROS_INTERFACE_ROS_INTERFACE_H
#define INCLUDED_ROS_INTERFACE_ROS_INTERFACE_H

#include <ros_interface/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace ros_interface {

    /*!
     * \brief <+description of block+>
     * \ingroup ros_interface
     *
     */
    class ROS_INTERFACE_API ros_interface : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<ros_interface> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of ros_interface::ros_interface.
       *
       * To avoid accidental use of raw pointers, ros_interface::ros_interface's
       * constructor is in a private implementation
       * class. ros_interface::ros_interface::make is the public interface for
       * creating new instances.
       */
      static sptr make();
    };

  } // namespace ros_interface
} // namespace gr

#endif /* INCLUDED_ROS_INTERFACE_ROS_INTERFACE_H */

