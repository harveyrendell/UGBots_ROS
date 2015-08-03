class Robot

{
public:
	virtual int moveTo(int x, int y);
	virtual int stop(int time);
private:
	//velocity of the robot
	double linear_x;
	double angular_z;

	//pose of the robot
	double px;
	double py;
	double theta;	
}