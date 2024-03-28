#ifndef Vec3_H
#define Vec3_H
#include <cmath>

class Vec3 {
    public:
        double x, y, z;

        Vec3(){
            x = 0, y = 0, z = 0;
        }

        Vec3(double tx, double ty, double tz){
            x = tx, y = ty, z = tz;
        }
        
        double magnitude(){
            return sqrt(x*x + y*y + z*z);
        }

        Vec3 normalized(){
            return Vec3(x, y, z) / magnitude();
        }
        
        Vec3 cross(Vec3 vec){
            return Vec3(y*vec.z - z*vec.y, z*vec.x - x*vec.z, x*vec.y - y*vec.x);
        }
        
	double dot(Vec3 vec){
            return x*vec.x + y*vec.y + z*vec.z;
        }
        
	Vec3 operator + (const Vec3& a){
            return Vec3(x + a.x, y + a.y,z + a.z);
        }

	void operator += (const Vec3& a){
	    Vec3 ans = Vec3(x, y, z) + a;
	    x = ans.x, y = ans.y, z = ans.z;
	}
        
	Vec3 operator - (const Vec3& a){
            return Vec3(x - a.x, y - a.y, z - a.z);
        }

	void operator -= (const Vec3& a){
	    Vec3 ans = Vec3(x, y, z) - a;
	    x = ans.x, y = ans.y, z = ans.z;
	}
	
	Vec3 operator * (const double& s){ 
            return Vec3(x * s, y * s, z * s); 
        }

	void operator *= (const double& s){
	    Vec3 ans = Vec3(x, y, z) * s;
	    x = ans.x, y = ans.y, z = ans.z;
	}
	
	Vec3 operator / (const double& s){
	    return Vec3(x / s, y / s, z / s);
	}

	void operator /= (const double& s){
	    Vec3 ans = Vec3(x, y, z) / s;
	    x = ans.x, y = ans.y, z = ans.z;
	}
};

#endif
