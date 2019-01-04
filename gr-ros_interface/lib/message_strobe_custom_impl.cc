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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/block_detail.h>
#include "message_strobe_custom_impl.h"

namespace gr {
  namespace ros_interface {

  message_strobe_custom::sptr
  message_strobe_custom::make(pmt::pmt_t msg, float period_ms, unsigned long int msg_count)
  {
    return gnuradio::get_initial_sptr
      (new message_strobe_custom_impl(msg, period_ms, msg_count));
  }

  message_strobe_custom_impl::message_strobe_custom_impl(pmt::pmt_t msg, float period_ms, unsigned long int msg_count)
    : block("message_strobe_custom",
            io_signature::make(0, 0, 0),
            io_signature::make(0, 0, 0)),
      d_finished(false),
      d_period_ms(period_ms),
      d_msg(msg),
	  d_msg_count(msg_count),
	  d_sent_msgs(0)
  {
    message_port_register_out(pmt::mp("strobe"));

    message_port_register_in(pmt::mp("set_msg"));
    set_msg_handler(pmt::mp("set_msg"),
                    boost::bind(&message_strobe_custom_impl::set_msg, this, _1));
  }

  message_strobe_custom_impl::~message_strobe_custom_impl()
  {
  }

  bool
  message_strobe_custom_impl::start()
  {
    // NOTE: d_finished should be something explicitely thread safe. But since
    // nothing breaks on concurrent access, I'll just leave it as bool.
    d_finished = false;
    d_thread = boost::shared_ptr<gr::thread::thread>
      (new gr::thread::thread(boost::bind(&message_strobe_custom_impl::run, this)));

    return block::start();
  }

  bool
  message_strobe_custom_impl::stop()
  {
    // Shut down the thread
    d_finished = true;
    d_thread->interrupt();
    d_thread->join();

    return block::stop();
  }

  void message_strobe_custom_impl::run()
  {
    while(!d_finished) {
      boost::this_thread::sleep(boost::posix_time::milliseconds(d_period_ms));
      if(d_finished) {
        return;
      }

      if (d_sent_msgs < d_msg_count) {
		  message_port_pub(pmt::mp("strobe"), d_msg);
		  d_sent_msgs += 1;
      } else {
    	  //post(pmt::mp("system"), pmt::cons(pmt::mp("done"), pmt::PMT_T));
    	  detail().get()->set_done(true);
    	  d_finished = true;
      }
    }
  }

  } /* namespace ros_interface */
} /* namespace gr */

