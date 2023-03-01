#ifndef BetterKalmanFilter_H
#define BetterKalmanFilter_H
#include <BasicLinearAlgebra.h>

class BetterKalmanFilter {
    public:
        BLA::Matrix<3, 1> X;
        BLA::Matrix<3, 3> P;
        BLA::Matrix<1> R;
        BLA::Matrix<3, 3> I;
        BLA::Matrix<1, 3> H;

    BetterKalmanFilter();
    void predict(float dt);
    void update(float dt, float measurement);
};

#endif