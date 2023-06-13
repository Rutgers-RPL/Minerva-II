#ifndef ECEFUtils_H
#define ECEFUtils_H

#include "Quaternion.h"
#include "Vec3.h"
#include "Mat3x3.h"

class ECEFUtils{
    public:
        static const Quaternion dcm2quat(Vec3 north, Vec3 east, Vec3 down);
        static const Quaternion ned2ecef(Quaternion q, double lambda, double phi);
};

#endif