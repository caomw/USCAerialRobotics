//      timer.hpp
//      A class based on std::chrono for recording and desplaying elapsed time.
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

#ifndef _ART_COMMON_TIMER_
#define _ART_COMMON_TIMER_

#include <chrono>

using namespace std;

namespace art
{
	struct Timer
	{
		// Store the beginning time.
		monotonic_clock::time_point tmp;
		
		// Initialize the timer.
		Timer(){ reset(); }
		
		// Reset the timer.
		void reset(){ tmp = monotonic_clock::now(); }
		
		// Return the time elapsed.
		double span(){ return std::chrono::duration<double>(monotonic_clock::now() - tmp).count();}
		
		// Print out the time elapsed with a user-specified label.
		void echospan(string s1 = "Echo Span"){
			cout << "\n[ " << s1 << " ] " << span() << ".\n";
		}
	};
}

#endif
