#ifndef AHRS_H
#define AHRS_H
#include <Arduino.h>
#include <Vec3.h>
#include <Quaternion.h>

#define _g_ (9.80665)

class Ahrs{
    public:
        double cornerFrequency = 0.02;

        double wmin = 4*(PI/180); //threshold under which the gyro is considered stationary
        double tb = 3; //time threshold after which we activate gyro bias compensation
        double fb;//time spent stationary

        double ta = 0.1;
        double grange = 0.1;
        double fa = 0;

        double Knorm = 0.5;
        double Kinit = 10;
        double tinit = 3;

        Quaternion q = Quaternion();

        Quaternion aglobal = Quaternion();

        Vec3 gyroffset;

        long lastTime;

        Ahrs(){
            lastTime = micros();
        }

        void update(Vec3 acc, Vec3 gyr, Vec3 mag){
            
            // Scale acc to in terms of g
            acc = acc * (1/_g_);

            double time2 = micros();
            double delta = ((double)(time2-lastTime))/1000000;
            lastTime = time2;

            double K;
            if(time2/1000000 < tinit) K = Knorm + ((tinit-(time2/1000000))/tinit)*(Kinit - Knorm);
            else K = Knorm;

            Vec3 wprime = gyr + (gyroffset * (-1));
            // Reset timer if gyroscope not stationary
            if ((fabs(gyr.x) > wmin) || (fabs(gyr.y) > wmin) || (fabs(gyr.z) > wmin)) {
                fb=0;
            } else if (fb < tb) {
                fb+=1;
            } else {
                // Adjust offset if timer has elapsed
                gyroffset = gyroffset + gyroBiasCompensation(gyr*delta);
            }

            Vec3 mprime(0,0,0);
            if (mag.magnitude() > 22 && mag.magnitude() < 67){
                mprime = mag;
            }

            Vec3 aprime = acc;
            if(!(acc.magnitude() - 1 > -1 * grange && acc.magnitude() - 1 < grange)){ //the acceleration has NOT exceeded the range for gravity
                fa+=delta;
                if(fa > ta)
                    aprime = Vec3(0, 0, 0);
            } else {
                fa = 0;
            }

            Vec3 gainAdjustedw = wprime + (errorTerm(aprime, mprime, q)*K);
            Quaternion qdot = (q * 0.5) * Quaternion(gainAdjustedw.x, gainAdjustedw.y, gainAdjustedw.z);
            q += qdot * delta;
            q = q.normalize();

            Vec3 onegoffset = Vec3(2*(q.b*q.d)-2*(q.a*q.c), 
                                   2*(q.c*q.d)+2*(q.a*q.b), 
                                  (2*pow(q.a, 2.0))-1+(2*pow(q.d, 2.0))) * -1;

            //Serial.printf("onegoffset: %f, %f, %f, %f\n", onegoffset.x, onegoffset.y, onegoffset.z, onegoffset.magnitude());
            Vec3 azero = acc+(onegoffset);
            
            //Serial.printf("azero: %f, %f, %f\n", azero.x, azero.y, azero.z);
            aglobal = q.rotate(Quaternion(azero.x, azero.y, azero.z));
            aglobal = aglobal * _g_;
        }

        Vec3 errorTerm(Vec3 acc, Vec3 mag, Quaternion q){
            Vec3 ahat = acc;
            ahat.normalize();
            Vec3 ea = ahat.cross(Vec3(2*(q.b*q.d)-2*(q.a*q.c),
                                  2*(q.c*q.d)+2*(q.a*q.b),
                                  2*pow(q.a,2)-1+2*pow(q.d,2)));
            ea.normalize();
            Vec3 em = acc.cross(mag);
            em=em.cross(Vec3(2*(q.b*q.c)+2*(q.a*q.d),
                         2*pow(q.a,2)-1+2*pow(q.c,2),
                         2*(q.c*q.d)-2*(q.a*q.b)));
            em.normalize();

            Vec3 e(0,0,0);
            if(acc.magnitude() > 0 && mag.magnitude() > 0){
                e = ea + em;
                //e.normalize();
                //Serial.printf("ea: %f, %f, %f em: %f, %f, %f\n", ea.x, ea.y, ea.z, em.x, em.y, em.z);
            }else if(acc.magnitude() > 0) {
                e = ea;
                //e.normalize();
            }
            return e;
        }

        Vec3 gyroBiasCompensation(Vec3 gyr){ //only call if stationary. pass component of integrated gyro output (dw)
            return gyr * (2*PI*cornerFrequency);
        }
};
#endif