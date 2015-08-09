#include <point.h>

class Pose
{
public:
	void setPose(double x, double y);
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

	Point getPose()
	{
		
	}
private:
	double px;
	double py;
	double theta;	
};
