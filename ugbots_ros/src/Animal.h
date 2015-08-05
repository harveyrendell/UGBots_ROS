class Animal

{
public:
	void publish()
	{
		//messages to stage
		node_cmdvel.linear.x = linear_x;
		node_cmdvel.angular.z = angular_z;
        
		//publish the message
		node_stage_pub.publish(node_cmdvel);

		ros::spinOnce();
	}
};
