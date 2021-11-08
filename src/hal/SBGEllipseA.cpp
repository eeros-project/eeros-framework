#include <eeros/hal/SBGEllipseA.hpp>

using namespace eeros::hal;

Vector3 SBGEllipseA::eulerData, SBGEllipseA::accData, SBGEllipseA::gyroData;
Vector4 SBGEllipseA::quatData;
uint32_t SBGEllipseA::timestampEuler, SBGEllipseA::timestampQuat, SBGEllipseA::timestampAcc, SBGEllipseA::timestampGyro;
uint32_t SBGEllipseA::count, SBGEllipseA::count0, SBGEllipseA::countEuler, SBGEllipseA::countQuat, SBGEllipseA::countImu;


