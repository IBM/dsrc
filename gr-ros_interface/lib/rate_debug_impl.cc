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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/block_detail.h>
#include "rate_debug_impl.h"

namespace gr {
  namespace ros_interface {

    rate_debug::sptr
    rate_debug::make()
    {
      return gnuradio::get_initial_sptr
        (new rate_debug_impl());
    }

    /*
     * The private constructor
     */
    rate_debug_impl::rate_debug_impl()
      : gr::block("rate_debug",
    		  gr::io_signature::make(0, 0, 0),
			  gr::io_signature::make(0, 0, 0)),
		d_rcv_msgs(0),
		d_rcv_bytes(0)
    {
    	message_port_register_in(pmt::mp("in"));
    	set_msg_handler(pmt::mp("in"), boost::bind(&rate_debug_impl::update_rate, this, _1));
    	d_begin = std::chrono::high_resolution_clock::now();
    }

    rate_debug_impl::~rate_debug_impl()
    {
    }

    void
	rate_debug_impl::update_rate(pmt::pmt_t msg)
    {
    	if (pmt::is_eof_object(msg)) {
    		detail().get()->set_done(true);
    		return;
    	} else if(pmt::is_symbol(msg)) {
    		return;
    	}

    	msg = pmt::cdr(msg);
    	int data_len = pmt::blob_length(msg);

    	d_rcv_msgs++;
    	d_rcv_bytes+=data_len;

    	std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - d_begin;
    	double rate_bps = 8 * d_rcv_bytes / elapsed.count();

std::chrono::high_resolution_clock::time_point start;
std::chrono::high_resolution_clock::time_point current = std::chrono::high_resolution_clock::now();
unsigned int count_for_average;
if (d_rcv_msgs > 100) {
start = d_times_queue.front();
d_times_queue.pop();
d_rcv_queue_total = d_rcv_queue_total - d_rcv_bytes_queue.front() + data_len;
d_rcv_bytes_queue.pop();
count_for_average = 100;
} else {
start = d_begin;
d_rcv_queue_total = d_rcv_bytes;
count_for_average = d_rcv_msgs;
}
d_times_queue.push(current);
d_rcv_bytes_queue.push(data_len);
std::chrono::duration<double> elapsed_100 = current - start;
double rate_bps_100 = 8 * d_rcv_queue_total / elapsed_100.count();
double rate_pps_100 = count_for_average / elapsed_100.count();

    	std::cout << "Bytes received: " << data_len << std::endl;
    	std::cout << "Messages received: " << d_rcv_msgs << std::endl;
    	std::cout << "Elapsed seconds: " << elapsed.count() << std::endl;
    	std::cout << "Rate (bits/sec): " << rate_bps << std::endl;
std::cout << "Rate_bps_100:    " << rate_bps_100 << std::endl;
std::cout << "Rate_pps_100:    " << rate_pps_100 << std::endl;

    }

  } /* namespace ros_interface */
} /* namespace gr */

