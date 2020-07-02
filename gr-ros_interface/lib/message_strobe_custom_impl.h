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

#ifndef INCLUDED_ROS_INTERFACE_MESSAGE_STROBE_CUSTOM_IMPL_H
#define INCLUDED_ROS_INTERFACE_MESSAGE_STROBE_CUSTOM_IMPL_H

#include <ros_interface/message_strobe_custom.h>

namespace gr {
  namespace ros_interface {

  class ROS_INTERFACE_API message_strobe_custom_impl : public message_strobe_custom
  {
  private:
    boost::shared_ptr<gr::thread::thread> d_thread;
    bool d_finished;
    long int d_period_ms;
    pmt::pmt_t d_msg;
    unsigned long int d_msg_count;
    unsigned long int d_sent_msgs;

    void run();

  public:
    message_strobe_custom_impl(pmt::pmt_t msg, long int period_ms, unsigned long int msg_count);
    ~message_strobe_custom_impl();

    void set_msg(pmt::pmt_t msg) { d_msg = msg; }
    pmt::pmt_t msg() const { return d_msg; }
    void set_period(long int period_ms) { d_period_ms = period_ms; }
    float period() const { return d_period_ms; }
    void set_msg_count(unsigned long int msg_count) { d_msg_count = msg_count; }
    float msg_count() const { return d_msg_count; }


    // Overloading these to start and stop the internal thread that
    // periodically produces the message.
    bool start();
    bool stop();
  };

 /*
    class message_strobe_custom_impl : public message_strobe_custom
    {
     private:
      // Nothing to declare in this block.

     public:
      message_strobe_custom_impl();
      ~message_strobe_custom_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };
*/

  } // namespace ros_interface
} // namespace gr

#endif /* INCLUDED_ROS_INTERFACE_MESSAGE_STROBE_CUSTOM_IMPL_H */

