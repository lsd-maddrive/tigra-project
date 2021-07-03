#ifndef ODOM_MODEL_H_
#define ODOM_MODEL_H_

#include <cmath>

class OdometryBicycleModel
{
public:
    OdometryBicycleModel(double wheel_radius, double wheelbase) : x_(0), y_(0), yaw_(0), prev_ts_(-1), wheel_radius_(wheel_radius), wheelbase_(wheelbase)
    {
        rps2mps_ = wheel_radius;
    }

    void updateState(double steer_rad, double speed_rps, double ts);

    // double getX() { return x_ + wheelbase_ * (1.0 - cos(yaw_)); }
    double getX() { return x_; }
    // double getY() { return y_ - wheelbase_ * sin(yaw_); }
    double getY() { return y_; }
    double getYaw() { return yaw_; }

    double getVX() { return vx_; }
    double getWYaw() { return wyaw_; }

    void resetState() {
        x_ = 0;
        y_ = 0;
        yaw_ = 0;
    }

private:
    // 2D coordinates in global positioning [m]
    double x_;
    double y_;
    // Yaw angle [rad]
    double yaw_;

    // Linear speed [m/s]
    double vx_;
    // Angular speed [rad/s]
    double wyaw_;

    double prev_ts_;

    double wheel_radius_;
    double wheelbase_;

    double rps2mps_;

    const double MIN_CURVATURE_RADIUS = 1e-4;
};

#endif // ODOM_MODEL_H_
