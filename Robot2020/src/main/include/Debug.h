#ifndef DEBUG_H
#define DEBUG_H

//This line:
#define USE_DEBUG //Uncomment or comment to use or not use

#ifdef USE_DEBUG
#define debugDashSend(s,x) frc::SmartDashboard::PutData(s,x)
#define debugDashNum(s,x) frc::SmartDashboard::PutNumber(s,x)
#define debugCons(x) std::cout << x;
#else
#define debugDashSend(s,x)
#define debugDashNum(s,x)
#define debugCons(x)
#endif

#endif // DEBUG_H