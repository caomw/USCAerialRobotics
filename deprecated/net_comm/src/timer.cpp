#include <ros/ros.h>

#include "timer.hpp"

using namespace std;

namespace art
{
	// Return current time.
	double Timer::now()
	{
		stringstream ss;
		ss << ros::Time::now();
		double x;
		ss >> x;
		return x;
	}
	
	// Initialize the timer.
	Timer::Timer(){ reset(); }
	
	// Reset the timer.
	void Timer::reset(){ tmp = now(); }
	
	// Return the time elapsed.
	double Timer::span(){return (now() - tmp);}
	
	// Print out the time elapsed with a label.
	void Timer::echospan(string s1 = ""){
		cout << "\n[ " << s1 << " ] " << span() << ".\n";
	}
}
