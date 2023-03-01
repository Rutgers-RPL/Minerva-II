#ifndef KalmanFilter_H
#define KalmanFilter_H
#include <BasicLinearAlgebra.h>

class KalmanFilter {
    public:
        BLA::Matrix<9, 1> X;
        BLA::Matrix<9, 9> P;
        BLA::Matrix<4, 4> R;
        BLA::Matrix<9, 9> I;
        BLA::Matrix<4, 9> H;

    KalmanFilter();
    void predict(float dt, float accelerometer_x, float accelerometer_y, float accelerometer_z);
    void updateGPS(float dt, float gps_x, float gps_y, float gps_z);
    void updateBaro(float dt, float barometer_z);
};

#endif