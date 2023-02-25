#include "bkf.h"
BetterKalmanFilter::BetterKalmanFilter() {
    this->X = {0.0, 0.0, 0.0};
    this->P = {100.0, 0.0, 0.0,
         0.0, 10.0, 0.0,
         0.0, 0.0, 100.0};
    this->H = {1.0, 0.0, 0.0,
               0.0, 0.0, 1.0};
    this->R = {5.0, 0.0,
               0.0, 10.0};
    this->I = {1.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0};
}

void BetterKalmanFilter::predict(float dt) {
    BLA::Matrix<3, 3> Q = {dt*dt*dt*dt/4.0, dt*dt*dt/2.0, dt*dt/2.0,
                         dt*dt*dt/2.0, dt*dt, dt,
                         dt*dt/2.0, dt, 1.0};
    Q = Q * 2.35;

    BLA::Matrix<3, 3> F = {1.0, dt, 0.5*dt*dt,
                           0.0, 1.0, dt,
                           0.0, 0.0, 1.0};

    this->X = F * this->X;
    this->P = ((F * this->P) * ~F) + Q;
}
void BetterKalmanFilter::update(float dt, float position, float velocity, float acceleration) {
    BLA::Matrix<2> z = {position, acceleration};

    BLA::Matrix<2> y = z - (this->H * this->X);

    BLA::Matrix<2, 2> S = this->H * (this->P * ~H) + this->R;

    BLA::Invert(S);
    
    BLA::Matrix<3, 2> K = (this->P * ~H) * S;
    
    this->X = this->X + (K * y);

    this->P = (((this->I - (K * this->H)) * this->P) * ~(this->I - (K * this->H))) + ((K * this->R) * ~K);
}