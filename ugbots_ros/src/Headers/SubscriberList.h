Class SubscriberList
{
public:
	//Odometry subscriber
	ros::Subscriber sub_odom;
	//Laser subscriber
	ros::Subscriber sub_laser;
	//Publisher for publishing to stage
	ros::Publisher node_stage_pub;
}
