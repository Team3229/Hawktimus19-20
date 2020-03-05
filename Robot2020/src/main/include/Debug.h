// Author: Team 3229 Programming Subteam

#ifndef DEBUG_H
#define DEBUG_H

//This line:
#define USE_DEBUG //Uncomment or comment to use or not use

#ifdef USE_DEBUG
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#define debugDashSend(s,x) frc::SmartDashboard::PutData(s,x)
#define debugDashNum(s,x) frc::SmartDashboard::PutNumber(s,x)
#define debugCons(x) std::cout << x;
#define debug(x) x
#else
#define debugDashSend(s,x)
#define debugDashNum(s,x)
#define debugCons(x)
#define debug(x)
#endif

#endif // DEBUG_H