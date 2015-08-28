#include "ros/ros.h"
#include <stdlib.h>
#include <ugbots_ros/Position.h>
#include <ugbots_ros/robot_details.h>
#include <ugbots_ros/picker_row.h>
#include <node_structs/row.h>
#include <node_structs/robot_details.h>
#include <math.h> 

class Core
{
public:
	Core(ros::NodeHandle &n)
	{
		//initialise constant publishers and subscribers for the core
		world_layout = n.subscribe<ugbots_ros::Position>("/world_layout",1000, &Core::wl_callback, this);
		full_bin_list = n.subscribe<ugbots_ros::robot_details>("/full_bins",1000, &Core::fbl_callback, this);
		picker_list = n.subscribe<ugbots_ros::robot_details>("/idle_pickers",1000, &Core::pl_callback, this);
		carrier_list = n.subscribe<ugbots_ros::robot_details>("/idle_carriers",1000, &Core::cl_callback, this);
		rows_list = n.advertise<ugbots_ros::Position>("/row_loc",1000,true);
	}

	//call back function for the world layout topic subscriber
	void wl_callback(ugbots_ros::Position p) 
	{
		//Instantiate the new point of the beacons position using info from callback
		Point point;
		point.x = p.x;
		point.y = p.y;
		//Add the beacon to organise into list of rows and beacons
		addBeacon(point);
	}

	//call back function for the list of full bins
	void fbl_callback(ugbots_ros::robot_details b)
	{
		//Instantiate the position of bin and the picker robots namespace it belongs to
		RobotDetails bin;
		bin.x = b.x;
		bin.y = b.y;
		bin.ns = b.ns;

		//add the bin to the list of bins
		addBin(bin);
	}

	//call back function for the idle picker identifier topic subscriber
	void pl_callback(ugbots_ros::robot_details r) 
	{
		//Instantiate the new robot details object using information from the callback
		RobotDetails robot;
		robot.x = r.x;
		robot.y = r.y;
		robot.ns = r.ns;

		//add to the list of idle picker bots
		addIdlePickerBot(robot);
	}

	//call back function for the idle carrier identifier topic subscriber
	void cl_callback(ugbots_ros::robot_details r) 
	{
		//Instantiate the new robot details object using information from the callback
		RobotDetails robot;
		robot.x = r.x;
		robot.y = r.y;
		robot.ns = r.ns;

		//add to the list of idle carrier bots
		addIdleCarrierBot(robot);
	}

	//function managing the beacons and its relevent lists
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

	//function adding a row to the row list
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

 	void addBin(RobotDetails b)
 	{
 		bool exists = false;

 		//iterate through all the points in the list and check if the point exists within it
		for (std::vector<RobotDetails>::iterator bin = binPositions.begin(); bin != binPositions.end(); ++bin) {
			RobotDetails current = *bin;
			if (current.x == b.x && current.y == b.y) {
				exists = true;
				break;
			}
		}

		//if it doesn't exist append to the end of the list
		if (!exists)
		{
			binPositions.push_back(b);
		}
 	}

 	//function adding robot details into the idle picker bots list
 	void addIdlePickerBot(RobotDetails rd) {
 		//boolean to keep track if bot is already in list
 		bool bot_exists = false;

 		//iterate through the list of idle picker bots
 		for (std::vector<RobotDetails>::iterator robot = idlePickers.begin(); robot != idlePickers.end(); ++robot) {
 			RobotDetails current = *robot;
 			//set boolean as true if already existing
 			if (rd.ns.compare(current.ns) == 0) {
 				bot_exists = true;
 				break;
 			}
 		}

 		//if unique, add to the list of idle picker bots
 		if (!bot_exists) {
 			idlePickers.push_back(rd);
 		}
 	}

 	//function adding robot details into the idle carrier bots list
 	void addIdleCarrierBot(RobotDetails rd) {
 		//boolean to keep track if bot is already in list
 		bool bot_exists = false;

 		//iterate through the list of idle carrier bots
 		for (std::vector<RobotDetails>::iterator robot = idleCarriers.begin(); robot != idleCarriers.end(); ++robot) {
 			RobotDetails current = *robot;
 			//set boolean as true if already existing
 			if (rd.ns.compare(current.ns) == 0) {
 				bot_exists = true;
 				break;
 			}
 		}

 		//if unique, add to the list of idle picker bots
 		if (!bot_exists) {
 			idleCarriers.push_back(rd);
 		}
 	}

 	//Getter function to get beacon, idle picker & carriers lists
	std::vector<Point> getBeaconList()
	{
		return beaconPositions;
	}
	std::vector<RobotDetails> getPickerList()
	{
		return idlePickers;
	}
	std::vector<RobotDetails> getCarrierList()
	{
		return idleCarriers;
	}

	//function to assign an unassigned row to the closest idle picker robot
	void assignRowToClosest(ros::NodeHandle &n) {
		//index for the list of rows
		int rowNum = 0;

		//iterate through list of rows
		for (std::vector<Row>::iterator row = rowPositions.begin(); row != rowPositions.end(); ++row) {
			Row current = *row;
			//for only the rows that are unassigned
			if (current.status == Row::UNASSIGNED) {
				//have both start and end point as valid start points
				Point start;
				Point end;
				int i;
				//get index for the closest robot for both start and end point of the row
				int a = getClosestPicker(current.start_point);
				int b = getClosestPicker(current.end_point);
				//when none exists, break out
				if (a < 0 || b < 0) {
					break;
				}
				//Use the points of the picker bots a and b
				Point pa;
				Point pb;
				pa.x = idlePickers[a].x;
				pa.y = idlePickers[a].y;
				pb.x = idlePickers[b].x;
				pb.y = idlePickers[b].y;
				//select picker bot which is closer, whether it starts from end or start
				if (getDistance(pa, current.start_point) < getDistance(pb, current.end_point)) {
					i = a;
					start = current.start_point;
					end = current.end_point;
				} else {
					i = b;
					start = current.end_point;
					end = current.start_point;
				}
				//initialise into the type for the topic
				ugbots_ros::picker_row bots_row;
				bots_row.start_x = start.x;
				bots_row.start_y = start.y;
				bots_row.end_x = end.x;
				bots_row.end_y = end.y;
				//dynamically set up the publishing topic for the specfically selected bot
				std::string topic = idlePickers[i].ns + "/station";
				row_distributer = n.advertise<ugbots_ros::picker_row>(topic, 1000, true);
				//erase the picker bot off the idle picker bot list as its not idle anymore
				idlePickers.erase(idlePickers.begin() + i);
				//set the row as assigned
				rowPositions[rowNum].status = Row::ASSIGNED;
				//publish the target point (station) to the picker bots topic
				row_distributer.publish(bots_row);
				break;
			}
			rowNum++;
		}
	}

	//function to retrieve the index number of the robot details of an idle picker bot relative to a point
	int getClosestPicker(Point p) {
		//list index
		int i = 0;
		//index of closest
		int closest = -1;
		//distance of the current closest robot
		double closestDistance = -1;

		//iterate through list of idle picker bots
		for (std::vector<RobotDetails>::iterator robot = idlePickers.begin(); robot != idlePickers.end(); ++robot) {
			RobotDetails current = *robot;
			//set up the current robots position point
			Point robotsPoint;
			robotsPoint.x = current.x;
			robotsPoint.y = current.y;
			//get the distance between the target point and current robots position point
			double distanceForCurrentRobot = getDistance(p, robotsPoint);
			//if the current distance is less than stated closest distance, or if closest distance has not been set up yet
			if (closestDistance > distanceForCurrentRobot || closestDistance == -1) {
				//set new index for the closest and replace stated closest distance
				closest = i;
				closestDistance = distanceForCurrentRobot;
			}
			i++;
		}

		return closest;
	}

	//function to assign an unassigned row to the closest idle picker robot
	void assignBinToClosest(ros::NodeHandle &n) {
		//only assigned when there are bins to be carried
		if (binPositions.size() > 0) {
			//call for a carrier for the first bin in the list
			RobotDetails b = binPositions[0];
			//initialise into type for the topic
			ugbots_ros::robot_details binLoc;
			binLoc.x = b.x;
			binLoc.y = b.y;
			binLoc.ns = b.ns;

			//use point type to find closest carrier bots index
			Point p;
			p.x = b.x;
			p.y = b.y;
			int i = getClosestCarrier(p);

			//if -1, there is no carrier bots for exit out of function
			if (i < 0) {
				return;
			}

			//dynamically assign topic to the specific carrier bot
			std::string topic = idleCarriers[i].ns + "/bin";
			bin_distributer = n.advertise<ugbots_ros::robot_details>(topic, 1000, true);
			//erase the carrier bot off the idle carrier bot list as its not idle anymore
			idleCarriers.erase(idleCarriers.begin() + i);
			//erase the bin off the list of full bins
			binPositions.erase(binPositions.begin());
			//publish the target point (bin location) to the carrier bots topic
			bin_distributer.publish(binLoc);
		}
	}

	//function to retrieve the index number of the robot details of an idle picker bot relative to a point
	int getClosestCarrier(Point p) {
		//list index
		int i = 0;
		//index of closest
		int closest = -1;
		//distance of the current closest robot
		double closestDistance = -1;

		//iterate through list of idle picker bots
		for (std::vector<RobotDetails>::iterator robot = idleCarriers.begin(); robot != idleCarriers.end(); ++robot) {
			RobotDetails current = *robot;
			//set up the current robots position point
			Point robotsPoint;
			robotsPoint.x = current.x;
			robotsPoint.y = current.y;
			//get the distance between the target point and current robots position point
			double distanceForCurrentRobot = getDistance(p, robotsPoint);
			//if the current distance is less than stated closest distance, or if closest distance has not been set up yet
			if (closestDistance > distanceForCurrentRobot || closestDistance == -1) {
				//set new index for the closest and replace stated closest distance
				closest = i;
				closestDistance = distanceForCurrentRobot;
			}
			i++;
		}

		return closest;
	}

	//calculates the distance between two points
	double getDistance(Point a, Point b) {
		return sqrt(pow((a.x - b.x),2) + pow((a.y - b.y),2));
	}

	//simple publisher for the visitors to know the location of rows
	void publishRows() {
		for (std::vector<Row>::iterator row = rowPositions.begin(); row != rowPositions.end(); ++row) {
			Row current = *row;
			ugbots_ros::Position p;
			p.x = current.x_pos;
			p.y = current.start_point.y;
			rows_list.publish(p);
		}
	}

	ros::Subscriber world_layout;
	ros::Subscriber full_bin_list;
	ros::Subscriber picker_list;
	ros::Subscriber carrier_list;
	ros::Publisher bin_distributer;
	ros::Publisher row_distributer;
	ros::Publisher rows_list;

private:
	std::vector<Point> beaconPositions;
	std::vector<Row> rowPositions;
	std::vector<RobotDetails> binPositions;
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

		//when count is 4, publish the rows for visitors, just once
		if (count == 4) {
			c.publishRows();
		}

		//once a count of over 5, start with the assignment
		//to prevent incorrect row recognition
		if (count > 5) {
			if (c.getPickerList().size() > 0) {
				c.assignRowToClosest(n);
			}
			if (c.getCarrierList().size() > 0) {
				c.assignBinToClosest(n);
			}
		}

		loop_rate.sleep();
		++count;
	}

	return 0;
}
