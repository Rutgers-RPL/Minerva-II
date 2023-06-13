#include "ECEFUtils.h"

#include <Quaternion.h>
#include "Vec3.h"
#include "Mat3x3.h"

const Quaternion ECEFUtils::dcm2quat(Vec3 north, Vec3 east, Vec3 down){
  // dcm2quat https://intra.ece.ucr.edu/~farrell/AidedNavigation/D_App_Quaternions/Rot2Quat.pdf
  int maximum = 1;
  double q = (1+north.x+east.y+down.z);
  double o = (1+north.x-east.y-down.z);
  if (o > q){
    q = o;
    maximum = 2;
  }
  o = (1-north.x+east.y-down.z);
  if (o > q){
    q = o;
    maximum = 3;
  }
  o = (1-north.x-east.y+down.z);
  if (o > q){
    q = o;
    maximum = 4;
  }
  
  Quaternion output = Quaternion();
  switch(maximum){
    case 1:
      output.a = 0.5*sqrt(q);
      output.b = (down.y-east.z)/(4*output.a);
      output.c = (north.z-down.x)/(4*output.a);
      output.d = (east.x-north.y)/(4*output.a);
      break;
    case 2:
      output.b = 0.5*sqrt(q);
      output.a = (down.y-east.z)/(4*output.b);
      output.c = (north.y+east.x)/(4*output.b);
      output.d = (north.z+down.x)/(4*output.b);
      break;
    case 3:
      output.c = 0.5*sqrt(q);
      output.a = (north.z-down.x)/(4*output.c);
      output.b = (north.y+east.x)/(4*output.c);
      output.d = (east.z+down.y)/(4*output.c);
      break;
    case 4:
      output.d = 0.5*sqrt(q);
      output.a = (east.x-north.y)/(4*output.d);
      output.b = (north.z+down.x)/(4*output.d);
      output.c = (east.z+down.y)/(4.0*output.d);
      break;
  }

  output.normalize();
  return output;
}

const Quaternion ECEFUtils::ned2ecef(Quaternion q, double lambda, double phi)
{
  Mat3x3 dcm =  Mat3x3( Vec3(-sin(lambda),            cos(lambda),            0),
                        Vec3(-cos(lambda)*sin(phi),   -sin(lambda)*sin(phi),  cos(phi)),
                        Vec3(cos(lambda)*cos(lambda), sin(lambda)*cos(phi),  sin(phi)));
  Quaternion rotation = dcm2quat(dcm.c1,dcm.c2,dcm.c3);
  return q * rotation;
}
