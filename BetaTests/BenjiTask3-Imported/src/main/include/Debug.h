// Author: Team 3229 Programming Subteam

#ifndef DEBUG_H
#define DEBUG_H

//This line:
//#define USE_DEBUG //Uncomment or comment to use or not use

#ifdef USE_DEBUG
#define debug(x) std::cout << x
#else
#define debug(x)
#endif

#endif // DEBUG_H
