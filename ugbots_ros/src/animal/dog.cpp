/**
	Author: UGBots

	The objects of this class freely roam around the orchard. 
	When obstacles are detected their change directions to avoid them.
	Their states regularly change between IDLE, WALKING, RUNNING and RANDOMTURN
	which reflects their linear and angular speed.
*/

#include <node_defs/dog.h>

Dog::Dog()
{
	//setting base attribute defaults
	this->pose.theta = M_PI/2.0;
	this->pose.px = 10;
	this->pose.py = 20;
	this->speed.linear_x = 0.0;
	this->speed.max_linear_x = 6.0;
	this->speed.angular_z = 0.0;
	this->orientation.currently_turning = false;

	//setup initial state.
	this->state = IDLE;
}

Dog::Dog(ros::NodeHandle &n)
{
	//setting base attribute defaults
	this->pose.theta = M_PI/2.0;
	this->pose.px = 10;
	this->pose.py = 20;
	this->speed.linear_x = 0.0;
	this->speed.max_linear_x = 6.0;
	this->speed.angular_z = 0.0;
	this->orientation.currently_turning = false;

	//setup initial state.
	this->state = IDLE;

	//register with neccessary topics
	this->sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, &Dog::odom_callback, this);
	this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000, &Dog::laser_callback, this);
	this->sub_list.sub_timer = n.createTimer(ros::Duration(5), &Dog::timerCallback, this);
}

void Dog::odom_callback(nav_msgs::Odometry msg)
{
	//update dogs position (pose) based on its position relative to world.		
	this->pose.px = msg.pose.pose.position.x;
	this->pose.py = msg.pose.pose.position.y;

	//assign,update orientation(x,y,z,w) values.
	this->orientation.rotx = msg.pose.pose.orientation.x;
	this->orientation.roty = msg.pose.pose.orientation.y;
	this->orientation.rotz = msg.pose.pose.orientation.z;
	this->orientation.rotw = msg.pose.pose.orientation.w;

	//print dogs position and status which will be picked up and update on GUI.
	ROS_INFO("/position/x/%f", this->pose.px);
	ROS_INFO("/position/y/%f", this->pose.py);
	ROS_INFO("/status/%s/./", enum_to_string(state));


	calculateOrientation(); //calculate orientation based on x,y,z,w values previously obtained.
	doAngleCheck(); //update dogs current angle and desired angle
	checkTurningStatus(); //check if dog is currently facing the direction its supposed to be facing.
	publish();

}


void Dog::laser_callback(sensor_msgs::LaserScan msg)
{
	bool detection = false; //declare a boolean value which will indicate whether there is an object in front of dog.
	if (this->orientation.currently_turning == false){ //detect for objects when not currently turning.
		for(int a = 0 ; a < 120; a++){
			//if there is something in front, turnback
			if ((msg.ranges[a] < 3) && (a > 80) && (a < 100)) {
				turnBack();
				break;
			//if there is something to the right, turn left
			} else if ((msg.ranges[a] < 3) && (a <= 80)) {
				turnLeft();
				break;
			//if there is something to the left, turn right
			} else if ((msg.ranges[a] < 3) && (a >= 100)){
				turnRight();
				break;
			}
			publish(); //publish here for quick reflexes of dog
		}
	}
}

//method that is called by ROS timer every 5 seconds
void Dog::timerCallback(const ros::TimerEvent& e){
	//update status of dog if it isn't turning.
	if (this->orientation.currently_turning == false){
		setStatus();
	}
}

//method which alters dogs Speed depending on its state.
void Dog::setStatus(){
	state = generateStatus();
	if (state == IDLE){
		stop();
	}else if (state == WALKING){
		walk();
	}else if (state == RUNNING){
		run();
	}else{
		//turn dogs orientation angle randomly
		turnRandomly();
	}
}

//Stops the node
void Dog::stop(){
	this->speed.linear_x = 0.0;
	this->speed.angular_z = 0.0;
}

//method which turns the dog into a random direction
void Dog::turnRandomly(){
	int randNum;
	srand (time(NULL));
	randNum = rand() % 6 + 1; //generate a random number between 1 and 6
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle + (randNum);
	this->speed.linear_x = 0.1;
	this->speed.angular_z = 1.0;
}

//makes dog walk
void Dog::walk(){
	this->speed.linear_x = 1.5;
	this->speed.angular_z = 0.0;
}

//makes dog run
void Dog::run(){
	this->speed.linear_x = 4.0;
	this->speed.angular_z = 0.0;
}

//Turn left
void Dog::turnLeft(){
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle + (M_PI / 2);
	this->speed.linear_x = 0.5;
	this->speed.angular_z = (M_PI/2);
}

//Turn right
void Dog::turnRight(){
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle - (M_PI / 2);
	this->speed.linear_x = 0.5;
	this->speed.angular_z = (-M_PI/2);
}

//Turn back
void Dog::turnBack(){
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle + (M_PI);
	this->speed.linear_x = 0.1;
	this->speed.angular_z = (M_PI/2);
}


//override checkTurningStatus() inorder to update dogs status everytime after turning so that it
//has a new speed to travel at.
void Dog::checkTurningStatus()
{
	if(this->orientation.currently_turning == true)
	{	
		if(doubleComparator(orientation.angle, orientation.desired_angle))
		{
			this->orientation.currently_turning = false;
			//reset dog's status after turning
			setStatus();
		}
	return;
	}
}

//method which turns state into string. Used to print states in ROS_INFO.
char const* Dog::enum_to_string(State t){
    switch(t){
        case WALKING:
            return "WALKING";
        case RUNNING:
            return "RUNNING";
        case IDLE:
            return "IDLE";
        case RANDOMTURN:
            return "RANDOMTURN";   
        default:
            return "INVALID ENUM";
    }
 }

//Returns a dogs status randomly.
Dog::State Dog::generateStatus(){
	int randNum;
	srand (time(NULL));
/* generate secret number between 1 and 5: */
	randNum = rand() % 5 + 1;
	if (randNum == 1){
		return IDLE;
	}else if (randNum == 2){
		return WALKING;
	}else if (randNum == 3){
		return RANDOMTURN;
	}else{
		return RUNNING;
	}
}

//unimplemented methods in the node interface.
void Dog::collisionDetected(){}
void Dog::move(){}

int main(int argc, char **argv)
{	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "DOG");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

Dog dg(n);

ros::Rate loop_rate(10);

//a count of howmany messages we have sent
int count = 0;

////messages
//velocity of this RobotNode
geometry_msgs::Twist RobotNode_cmdvel;

while (ros::ok())
{

	ros::spinOnce();

	loop_rate.sleep();
	++count;
}
return 0;

}
