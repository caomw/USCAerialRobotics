#include <ros/ros.h>
#include "pf.hpp"
#include "rm.hpp"
#include "sm.hpp"
#include "common.hpp"

int main()
{
	/** Init components **/
	RobotModel robot;
	SensorModel sensor;
	ParticleFilter filter;
	
	/** Init params **/
	float cmd_speed = 0.05;
	float cmd_rotation = 0.01;
	robot.setposition(0.0, -1.0, PI/3.0); // reset robot position
	robot.setvariance(0.1, 0.05);
	sensor.setvariance(0.1, 0.01, 0.0001);
	filter.setR(sensor.cov);
	
	/** Create particles **/
	for(int i = 0; i < 100; i++)
		filter.particles.push_back(Particle(robot.pos))
	
	/** Create landmarks **/
	vector<Vector3f> landmarks;
	landmarks.push_back(Vector3f(1.0, 3.0, 0.0));
	landmarks.push_back(Vector3f(2.0, 2.5, 0.0));
	landmarks.push_back(Vector3f(0.0, 3.4, 0.0));
	landmarks.push_back(Vector3f(0.0, 1.5, 0.0));
	landmarks.push_back(Vector3f(1.0, 3.5, 0.0));
	
	/** Main loop **/
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		/** Move the robot **/
		robot.move(cmd_speed, cmd_rotation);
		
		/** Take measurements **/
		vector<MeasureResult> results_measure;
		for(int i = 0; i < landmarks.size(); i++)
		{
			MeasureResult new_measure;
			if(!sensor.measure(robot.pos, landmarks[i], new_measure.result, new_measure.jacobian))
				continue;
			new_measure.id = i;
			results_measure.push_back(new_measure);
		}
		
		/** Process measurements in filter and update the particles **/
		filter.process_measure(results_measure);
		filter.resample();

		/** TODO: plot **/
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
