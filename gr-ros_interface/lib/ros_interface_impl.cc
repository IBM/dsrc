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

#include <string>
#include <fstream>
#include <pthread.h>
#include <gnuradio/io_signature.h>
#include "ros_interface_impl.h"
#include "lz4.h"
#include "nav_msgs/OccupancyGrid.h"

#define PREFIX_SIZE 4	// bytes

using namespace std;
using namespace ros;

namespace gr {
  namespace ros_interface {

    ros_interface::sptr
    ros_interface::make()
    {
      // Create and initialize ROS node
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "ros_interface");

      return gnuradio::get_initial_sptr
        (new ros_interface_impl());
    }

    ros_interface_impl::ros_interface_impl()
      : gr::block("ros_interface",
    		  gr::io_signature::make(0, 0, 0),
			  gr::io_signature::make(0, 0, 0)),
		_sent_msgs(0),
		_received_msgs(0),
		_spinner(1)
    {
      // Create input and output message ports
      message_port_register_in(PDU_PORT_ID);
      set_msg_handler(PDU_PORT_ID, boost::bind(&ros_interface_impl::handle_msg, this, _1));
      message_port_register_out(PDU_PORT_ID);

      _loc_map_sub = _n.subscribe<era_gazebo::ERAOccupancyGrid>("local_map", 10, &ros_interface_impl::handle_ros_topic, this);
      _rem_map_pub = _n.advertise<era_gazebo::ERAOccupancyGrid>("remote_map", 10);

      _spinner.start();
    }

    /*
     * Our virtual destructor.
     */
    ros_interface_impl::~ros_interface_impl()
    {
    }

    int
    ros_interface_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
    }

    void
    ros_interface_impl::handle_ros_topic(const era_gazebo::ERAOccupancyGrid msg)
    {
    	// ROS message is serialized
    	ros::SerializedMessage ser_msg = ros::serialization::serializeMessage(msg);
    	const uint32_t serial_size = ser_msg.num_bytes;

    	// Message is compressed
    	const int max_dst_size = LZ4_compressBound(serial_size);
    	char* compressed_data = (char *)malloc(PREFIX_SIZE+max_dst_size); // We allocate some prefixing bytes to include
    	                                                                  // some metadata (sizes) needed at the receiver
    	                                                                  // to decompress and deserialize the message.
    	if (compressed_data == NULL)
    	{
          ROS_FATAL("Failed to allocate memory for compressed data");
          return;
    	}
    	const int compressed_data_size = LZ4_compress_default((char *)ser_msg.message_start, &compressed_data[PREFIX_SIZE], serial_size, max_dst_size);
    	if (compressed_data_size <= 0)
    	{
          ROS_FATAL("Failed to compress data");
          free(compressed_data);
          return;
    	}
    	ROS_INFO("Compressing %d bytes into %d bytes", serial_size, compressed_data_size);

    	((uint32_t *)compressed_data)[0] = serial_size;

    	pmt::pmt_t pdu = pmt::cons(pmt::PMT_NIL, pmt::init_u8vector(PREFIX_SIZE+compressed_data_size, (uint8_t *)compressed_data));
    	message_port_pub(PDU_PORT_ID, pdu);

    	_sent_msgs += 1;
    }

    void
    ros_interface_impl::handle_msg(pmt::pmt_t pdu)
    {
    	const vector<uint8_t> compressed_data = pmt::u8vector_elements(pmt::cdr(pdu));

    	const uint32_t serial_size = ((uint32_t *)(&compressed_data[0]))[0];

    	// Message is uncompressed
    	char* serialized_payload = (char *)malloc(serial_size);
    	if (serialized_payload == NULL)
    	{
          ROS_FATAL("Failed to allocate memory for decompressed data");
          return;
    	}
    	const int decompressed_size = LZ4_decompress_safe((char *)(&compressed_data[PREFIX_SIZE]), serialized_payload, compressed_data.size()-PREFIX_SIZE, serial_size);
    	if (decompressed_size <= 0)
    	{
          ROS_FATAL("Failed to uncompress data (returned %d)", decompressed_size);
          free(serialized_payload);
          return;
    	}
    	// decompressed_size should be equal to serial_size
    	ROS_INFO("Uncompressing %lu bytes into %d bytes", compressed_data.size()-PREFIX_SIZE, decompressed_size);

    	// Message is deserialized
    	boost::shared_array<uint8_t> buffer((uint8_t *)serialized_payload);
    	ros::SerializedMessage ser_msg(buffer, serial_size);
    	era_gazebo::ERAOccupancyGrid msg;
    	ros::serialization::deserializeMessage(ser_msg, msg);
    	_rem_map_pub.publish(msg);

    	_received_msgs += 1;
    }


  } /* namespace ros_interface */
} /* namespace gr */

