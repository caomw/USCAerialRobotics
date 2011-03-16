//      debug.hpp
//      Common functionalities for debugging use.
//
//      Copyright (C) 2011 Sam (Yujia Zhai) <yujia.zhai@usc.edu>
//      Aerial Robotics Team, USC Robotics Society - http://www.uscrs.org - http://uscrs.googlecode.com
//
//      This program is free software; you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation; either version 2 of the License, or
//      (at your option) any later version.
//      
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
//      
//      You should have received a copy of the GNU General Public License
//      along with this program; if not, write to the Free Software
//      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
//      MA 02110-1301, USA.

#ifndef _DEBUG_HPP_
#define _DEBUG_HPP_

#include "ros/ros.h"
#include <sstream>

using namespace std;

// Using namespace for definitions in header files.
namespace beohawk
{
	struct Timer // using struct because its members and methods are public by default.
	{
		// Store the beginning time.
		double tmp;
		
		// Return current time.
		double now()
		{
			stringstream ss;
			ss << ros::Time::now();
			double x;
			ss >> x;
			return x;
		}
		
		// Initialize the timer.
		Timer(){ reset(); }
		
		// Reset the timer.
		void reset(){ tmp = now(); }
		
		// Return the time elapsed.
		double span(){ return (now() - tmp);}
		
		// Print out the time elapsed with a label.
		void echospan(string s1){
			cout << "\n[ " << s1 << " ] " << span() << ".\n";
		}
	};
}

#endif
