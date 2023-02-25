#include "kf.h"
KalmanFilter::KalmanFilter() {
    this->X = {0.0, 0.0, 0.0};
    this->Xp = {0.0, 0.0, 0.0};
    this->P = {10.0, 0.0, 0.0,
         0.0, 10.0, 0.0,
         0.0, 0.0, 10.0};
    this->Pp = {10.0, 0.0, 0.0,
         0.0, 10.0, 0.0,
         0.0, 0.0, 10.0};
    this->R = {1.0};
    I = {1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0};
    this->H = {1.0, 0.0, 0.0};
}

void KalmanFilter::predict(float dt, float acceleration) {
    BLA::Matrix<3, 3> F = {1.0, dt, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 0.0};

    BLA::Matrix<3, 3> B = {0.0, 0.0, 0.5 * dt * dt,
                           0.0, 0.0, dt,
                           0.0, 0.0, 1.0};

    BLA::Matrix<3> u = {0.0, 0.0, acceleration};

    BLA::Matrix<3, 3> Q = {dt*dt*dt*dt/4.0, dt*dt*dt/2.0, dt*dt/2.0,
                         dt*dt*dt/2.0, dt*dt, dt,
                         dt*dt/2.0, dt, 1.0};
    Q = Q * 0.02;

    this->X = F * this->Xp + B * u;
    this->P = F * this->Pp * ~F + Q;
    this->Xp = this->X;
    this->Pp = this->P;
}
void KalmanFilter::update(float dt, float position) {
    BLA::Matrix<1> pos = {position};
    BLA::Matrix<1> y = pos - this->H * this->Xp;
    BLA::Matrix<1> S = this->H * (this->Pp * ~this->H) + this->R;
    BLA::Matrix<3> K = this->Pp * (~this->H * BLA::Invert(S));

    this->X = this->Xp + K * y;
    this->P = (this->I - K * this->H) * (this->Pp * ~(this->I - K * this->H) + K * (this->R * ~K));
    Serial.println(X(0,0));
    this->Xp = this->X;
    this->Pp = this->P;
}