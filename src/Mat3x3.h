#ifndef Mat3x3_H
#define Mat3x3_H
#include "Vec3.h"
class Mat3x3{
    public:
        Vec3 c1;
        Vec3 c2;
        Vec3 c3;
        bool vertical = true;

        Mat3x3(){
        }

        Mat3x3(Vec3 tc1, Vec3 tc2, Vec3 tc3){
            c1 = tc1;
            c2 = tc2;
            c3 = tc3;
        }

        Vec3 operator * (const Vec3& v){ 
            return Vec3(c1.x * v.x + c2.x * v.y + c3.x * v.z, 
                        c1.y * v.x + c2.y * v.y + c3.y * v.z, 
                        c1.z * v.x + c2.z * v.y + c3.z * v.z); 
        }

};
#endif