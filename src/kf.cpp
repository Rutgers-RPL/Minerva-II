#include "kf.h"
KalmanFilter::KalmanFilter() {
    this->X = {0.0, 0.0, 0.0};
    this->P = {100.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 100.0};
    this->H = {1.0, 0.0, 0.0};
    this->R = {5.0};
    this->I = {1.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0};
}

void KalmanFilter::predict(float dt) {
    BLA::Matrix<3, 3, float> Q = {dt*dt*dt*dt/4.0, dt*dt*dt/2.0, dt*dt/2.0,
                         dt*dt*dt/2.0, dt*dt, dt,
                         dt*dt/2.0, dt, 1.0};
    Q = Q * 0.02f;

    BLA::Matrix<3, 3> F = {1.0, dt, 0.5*dt*dt,
                           0.0, 1.0, dt,
                           0.0, 0.0, 1};

    this->X = F * this->X;
    this->P = ((F * this->P) * ~F) + Q;
}
void KalmanFilter::update(float dt, float measurement) {
    BLA::Matrix<1> z = {measurement};

    BLA::Matrix<1> y = z - this->H * this->X;

    BLA::Matrix<1> S = this->H * (this->P * ~H) + this->R;

    BLA::Matrix<3> K = (this->P * ~H) * BLA::Inverse(S);
    
    this->X = this->X + (K * y);

    this->P = (((this->I - (K * this->H)) * this->P) * ~(this->I - (K * this->H))) + ((K * this->R) * ~K);
}