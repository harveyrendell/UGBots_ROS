class SubscriberList
{
public:
	//Odometry subscriber
	ros::Subscriber sub_odom;
	//Laser subscriber
	ros::Subscriber sub_laser;
	//Publisher for publishing to stage
	ros::Publisher node_stage_pub;
	//Publisher for publishing to debug node
	ros::Publisher node_debug_pub;
	//Subscriber for debug node
	ros::Subscriber sub_debug;
	//Subscriber for communication between carrier/picker
	ros::Subscriber sub_comm;
	//timer subscribe
	ros::Timer sub_timer;
};
