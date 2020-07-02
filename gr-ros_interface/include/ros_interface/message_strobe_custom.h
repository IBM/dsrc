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


#ifndef INCLUDED_ROS_INTERFACE_MESSAGE_STROBE_CUSTOM_H
#define INCLUDED_ROS_INTERFACE_MESSAGE_STROBE_CUSTOM_H

#include <ros_interface/api.h>
#include <gnuradio/block.h>
//#include <gnuradio/sync_block.h>

namespace gr {
  namespace ros_interface {

  /*!
   * \brief Send message at defined interval
   * \ingroup message_tools_blk
   *
   * \details
   * Takes a PMT message and sends it out \p msg_count times every \p period_ms
   * milliseconds. Useful for testing/debugging the message system.
   */
  class ROS_INTERFACE_API message_strobe_custom : virtual public block
  {
  public:
    // gr::blocks::message_strobe::sptr
    typedef boost::shared_ptr<message_strobe_custom> sptr;

    /*!
     * Make a message strobe block to send message \p msg every \p
     * period_ms milliseconds \p msg_count times.
     *
     * \param msg the message to send as a PMT.
     * \param period_ms the time period in milliseconds in which to
     *                  send \p msg.
     * \param msg_count the number of messages to send.
     */
    static sptr make(pmt::pmt_t msg, long int period_ms, unsigned long int msg_count);

    /*!
     * Reset the message being sent.
     * \param msg The message to send as a PMT.
     */
    virtual void set_msg(pmt::pmt_t msg) = 0;

    /*!
     * Get the value of the message being sent.
     */
    virtual pmt::pmt_t msg() const = 0;

    /*!
     * Reset the sending interval.
     * \param period_ms the time period in milliseconds.
     */
    virtual void set_period(long int period_ms) = 0;

    /*!
     * Get the time interval of the strobe.
     */
    virtual float period() const = 0;

    /*!
     * Reset the number of messages to send.
     * \param msg_count message count.
     */
    virtual void set_msg_count(unsigned long int msg_count) = 0;

    /*!
     * Get the number of messages to send.
     */
    virtual float msg_count() const = 0;

  };

  } // namespace ros_interface
} // namespace gr

#endif /* INCLUDED_ROS_INTERFACE_MESSAGE_STROBE_CUSTOM_H */

