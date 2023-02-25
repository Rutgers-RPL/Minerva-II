#ifndef KalmanFilter_H
#define KalmanFilter_H
#include <BasicLinearAlgebra.h>

class KalmanFilter {
    public:
        BLA::Matrix<3, 1> X;
        BLA::Matrix<3, 1> Xp;
        BLA::Matrix<3, 3> P;
        BLA::Matrix<3, 3> Pp;
        BLA::Matrix<1> R;
        BLA::Matrix<3, 3> I;
        BLA::Matrix<1, 3> H;

    KalmanFilter();
    void predict(float dt, float acceleration);
    void update(float dt, float position);
};

#endif