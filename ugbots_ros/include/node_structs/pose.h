class Pose
{
public:
	void setX(double x)
	{
		px = x;
	}

	void setY(double y)
	{
		py = z;
	}

	void setTheta(double t)
	{
		theta = z;
	}	

	double getpx()
	{
		return  px;
	}

	double getpy()
	{
		return py;
	}

	double getTheta()
	{
		return theta;
	}
	
private:
	double px;
	double py;
	double theta;	
};
