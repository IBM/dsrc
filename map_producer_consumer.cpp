#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>

using namespace std;

void callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	ROS_INFO("%s: Message received!", ros::this_node::getName().c_str());
	//ROS_INFO("%s: I heard [%s]", ros::this_node::getName().c_str(), msg);
}

int main(int argc, char **argv)
{
	/*** Occupancy grip map creation and initialization ***/
	int width  = 2;
	int height = 2;
	nav_msgs::OccupancyGrid map;
	map.info.resolution = 1.0;         // float32
	map.info.width      = width;       // uint32
	map.info.height     = height;      // uint32
	vector<signed char> data(width*height);
	map.data = data;

	/*** ROS node creation ***/
	ros::init(argc, argv, "map_producer_consumer");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 10);
	ros::Subscriber sub = n.subscribe("/map", 1000, callback);
	ros::Rate loop_rate(0.25); // In Hz

	while (ros::ok())
	{
		pub.publish(map);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
