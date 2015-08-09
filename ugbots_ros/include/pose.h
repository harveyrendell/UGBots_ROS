class Pose
{
public:
	void setPose(double x, double y)
	{
		px = x;
		py = y;
	}

	void setPose(double x, double y, double t)
	{
		px = x;
		py = y;
		theta = t;
	}

	double getpx()
	{
		return this->px;
	}

	double getpy()
	{
		return this->py;
	}

	double getTheta()
	{
		return this->theta;
	}

//private:
	double px;
	double py;
	double theta;	
};
