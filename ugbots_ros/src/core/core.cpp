#include "ros/ros.h"
#include <stdlib.h>
#include <ugbots_ros/Position.h>
#include <node_structs/row.h>

class Core
{
public:
	Core(ros::NodeHandle &n)
	{
		world_layout = n.subscribe<ugbots_ros::Position>("/world_layout",1000, &Core::wl_callback, this);
	}

	void wl_callback(ugbots_ros::Position p) 
	{
		Point point;
		point.x = p.x;
		point.y = p.y;
		addBeacon(point);
	}

	void addBeacon(Point p)
	{
		bool exists = false;

		for (std::vector<Point>::iterator point = beaconPositions.begin(); point != beaconPositions.end(); ++point) {
			Point current = *point;
			if (current.x == p.x && current.y == p.y) {
				exists = true;
				break;
			}
		}
		
		if (!exists)
		{
			beaconPositions.push_back(p);
		}
	}

	std::vector<Point> getBeaconList()
	{
		return beaconPositions;
	}

	ros::Subscriber world_layout;

private:
	std::vector<Point> beaconPositions;
	std::vector<Row> rowPositions;
};

int main(int argc, char **argv)
{
	// initialise node
	ros::init(argc, argv, "CORE");
	ros::NodeHandle n;

	Core c(n);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}
