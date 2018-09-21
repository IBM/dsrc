#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std;

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		ROS_ERROR("Usage: %s <pcap_pipe_filename>", argv[0]);
		return 1;
	}
	string pcap_pipe_filename = argv[1];

	int fd = 0;
	errno  = 0;
	struct stat st;

	fd = open(pcap_pipe_filename.c_str(), O_RDONLY | O_NONBLOCK);

	if (fd != -1)
	{
		// File already exists.
		fstat(fd, &st);
		if (!S_ISFIFO(st.st_mode))
		{
			// File exists and is not a FIFO (error!).
			close(fd);
			ROS_ERROR("File %s already exists but it's not a named pipe!", pcap_pipe_filename.c_str());
			return 1;
		}
		ROS_INFO("File %s exists and it's a pipe.", pcap_pipe_filename.c_str());
		close(fd);
	}
	else
	{
		// File doesn't exist.
		ROS_INFO("File %s doesn't exists. Creating it.", pcap_pipe_filename.c_str());
		mkfifo(pcap_pipe_filename.c_str(), 0777);
	}

	/*** ROS node creation ***/
	ros::init(argc, argv, "wireshark_wrapper");
	ros::NodeHandle n;

	string cmd = "wireshark -k -i " + pcap_pipe_filename;
	std::system(cmd.c_str());

	return 0;
}
