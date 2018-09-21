#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#define BUFSIZE 30000  // bytes

using namespace std;

void print_hex(unsigned char *s)
{
	printf("[");
	while(*s)
		printf("%02x", (unsigned int) *s++);
	printf("]\n");
}

int main(int argc, char **argv)
{
	if (argc < 3)
	{
		ROS_ERROR("Usage: %s <inbound_port> <outbound_port>", argv[0]);
		return 1;
	}

	/*** Socket creation/connection ***/
	int inbound_port  = atoi(argv[1]);
	int outbound_port = atoi(argv[2]);
	int inbound_sock, outbound_sock;
    struct sockaddr_in servaddr, cliaddr;
	int recvlen, sentlen;
	unsigned char buf[BUFSIZE];

	if ((inbound_sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		ROS_ERROR("Could not create inbound socket");
		exit(1);
	}
	if ((outbound_sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		ROS_ERROR("Could not create outbound socket");
		exit(1);
	}

	memset(&servaddr, 0, sizeof(servaddr));
	memset(&cliaddr, 0, sizeof(cliaddr));

	servaddr.sin_addr.s_addr = INADDR_ANY;
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(inbound_port);

	cliaddr.sin_addr.s_addr = INADDR_ANY;
	cliaddr.sin_family = AF_INET;
	cliaddr.sin_port = htons(outbound_port);

	// This option is needed on the socket in order to be able to receive broadcast messages
	int broadcast = 1;
	if (setsockopt(inbound_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
	{
		ROS_ERROR("Error setting the broadcast option");
		exit(1);
	}

	// Bind the socket with the server address
	if ( bind(inbound_sock, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 )
	{
		ROS_ERROR("inbound_sock bind failed");
		exit(1);
	}

	/*** ROS node creation ***/
	ros::init(argc, argv, "remote_agent");
	ros::NodeHandle n;
	ros::Rate loop_rate(1); // In Hz

	ROS_INFO("Waiting on port %d", inbound_port);

	while (ros::ok())
	{
		recvlen = recv(inbound_sock, buf, BUFSIZE, 0);

		if (recvlen > 0)
		{
			buf[recvlen] = 0;
			ROS_INFO("Received message (length: %d bytes):", recvlen);
			//print_hex(buf);

			sentlen = sendto(outbound_sock, (const char *)buf, recvlen, 0,
					(const struct sockaddr *) &cliaddr, sizeof(cliaddr));
			ROS_INFO("%d bytes sent back", sentlen);
		}

		//loop_rate.sleep();
	}

	return 0;
}
