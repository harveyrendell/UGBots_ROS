#include "ros/ros.h"
#include <stdlib.h>
#include <ugbots_ros/Position.h>
#include <ugbots_ros/robot_details.h>
#include <node_structs/row.h>
#include <node_structs/robot_details.h>
#include <math.h> 

class Core
{
public:
	Core(ros::NodeHandle &n)
	{
		world_layout = n.subscribe<ugbots_ros::Position>("/world_layout",1000, &Core::wl_callback, this);
		picker_list = n.subscribe<ugbots_ros::robot_details>("/tell_core",1000, &Core::pl_callback, this);
	}

	void pl_callback(ugbots_ros::robot_details r) 
	{
		RobotDetails robot;
		robot.x = r.x;
		robot.y = r.y;
		robot.ns = r.ns;
		addIdlePickerBot(robot);

		ROS_INFO("Number of idle picker bots: %d, %s", idlePickers.size(), robot.ns.c_str());
	}

	void wl_callback(ugbots_ros::Position p) 
	{
		Point point;
		point.x = p.x;
		point.y = p.y;
		addBeacon(point);

		ROS_INFO("Number of Rows: %d", rowPositions.size());
	}

	void addBeacon(Point p)
	{
		//boolean to check if the list has the point
		bool exists = false;

		//iterate through all the points in the list and check if the point exists within it
		for (std::vector<Point>::iterator point = beaconPositions.begin(); point != beaconPositions.end(); ++point) {
			Point current = *point;
			if (current.x == p.x && current.y == p.y) {
				exists = true;
				break;
			}
		}

		//if it doesn't exist append to the end of the list
		if (!exists)
		{
			beaconPositions.push_back(p);
			addPointToRow(p);
		}
	}

	void addPointToRow(Point p)
	{
		//boolean to check if the x coordinate for the row exists
		bool x_exists = false;
		//loops counter to access list element
		int i = 0;
		//initialise new row for the list of rows
		Row new_row;
		new_row.status = Row::UNASSIGNED;

		//iterate through existing list of rows
		for (std::vector<Row>::iterator row = rowPositions.begin(); row != rowPositions.end(); ++row) {
			Row current = *row;
			//when corresponding x coordinate found
			if (current.x_pos == p.x) {
				x_exists = true;
				//if the existing start points y coordinate is greater than the new points
				//replaces the row with appropriate start/end point
				if (current.start_point.y > p.y) {
					new_row.start_point = p;
					new_row.end_point = current.start_point;
					new_row.x_pos = p.x;
					rowPositions[i] = new_row;
				//if not, new point is the end point
				} else {
					new_row.start_point = current.start_point;
					new_row.end_point = p;
					new_row.x_pos = p.x;
					rowPositions[i] = new_row;
				}
				break;
			}
			i++;
		}

		//if the x coordinate does not exist, add to row as start point
		if (!x_exists) {
			new_row.start_point = p;
			new_row.x_pos = p.x;
			rowPositions.push_back(new_row);
		}
 	}

 	void addIdlePickerBot(RobotDetails rd) {
 		bool bot_exists = false;

 		for (std::vector<RobotDetails>::iterator robot = idlePickers.begin(); robot != idlePickers.end(); ++robot) {
 			RobotDetails current = *robot;
 			if (rd.ns.compare(current.ns) == 0) {
 				bot_exists = true;
 				break;
 			}
 		}

 		if (!bot_exists) {
 			idlePickers.push_back(rd);
 		}
 	}

	std::vector<Point> getBeaconList()
	{
		return beaconPositions;
	}

	void assignRowToClosest(ros::NodeHandle &n) {
		int rowNum = 0;

		for (std::vector<Row>::iterator row = rowPositions.begin(); row != rowPositions.end(); ++row) {
			Row current = *row;
			if (current.status == Row::UNASSIGNED) {
				Point p = current.start_point;
				ugbots_ros::Position station;
				station.x = p.x;
				station.y = p.y;
				int i = getClosestRobot(p);
				if (i < 0) {
					break;
				}
				std::string topic = idlePickers[i].ns + "/station";
				work_distributer = n.advertise<ugbots_ros::Position>(topic, 1000, true);
				ROS_INFO("station appointed to idle bot: %d, ns: %s", i, topic.c_str());
				idlePickers.erase(idlePickers.begin() + i);
				work_distributer.publish(station);
				rowPositions[rowNum].status = Row::ASSIGNED;
				break;
			}
			rowNum++;
		}
	}

	int getClosestRobot(Point p) {
		int i = 0;
		int closest = -1;
		double closestDistance = 100;

		for (std::vector<RobotDetails>::iterator robot = idlePickers.begin(); robot != idlePickers.end(); ++robot) {
			RobotDetails current = *robot;
			Point robotsPoint;
			robotsPoint.x = current.x;
			robotsPoint.y = current.y;
			double distanceForCurrentRobot = getDistance(p, robotsPoint);
			if (closestDistance > distanceForCurrentRobot) {
				closest = i;
			}
			i++;
		}

		return closest;
	}

	double getDistance(Point a, Point b) {
		return sqrt(pow((a.x - b.x),2) + pow((a.y - b.y),2));
	}

	ros::Subscriber world_layout;
	ros::Subscriber picker_list;
	ros::Publisher work_distributer;

private:
	std::vector<Point> beaconPositions;
	std::vector<Row> rowPositions;
	std::vector<RobotDetails> idlePickers;
	std::vector<RobotDetails> idleCarriers;
};

int main(int argc, char **argv)
{
	// initialise node
	ros::init(argc, argv, "CORE");
	ros::NodeHandle n;

	Core c(n);

	ros::Rate loop_rate(1);

	int count = 0;
	while (ros::ok())
	{

		ros::spinOnce();

		if (count > 5) {
			c.assignRowToClosest(n);
		}

		loop_rate.sleep();
		++count;
	}

	return 0;
}
