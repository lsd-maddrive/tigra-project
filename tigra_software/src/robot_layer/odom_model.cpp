#include <robot_layer/odom_model.hpp>

#include <ros/ros.h>

void OdometryBicycleModel::updateState(double steer_rad, double speed_rps, double ts)
{
    if (prev_ts_ < 0)
    {
        prev_ts_ = ts;
        return;
    }

    const double dt = ts - prev_ts_;
    prev_ts_ = ts;
    if (dt < 0.0001)
    {
        ROS_WARN("OdometryModel: Time delta is negative or too small - update skipped");
        return;
    }

    vx_ = speed_rps * rps2mps_;

    const double linear_diff = vx_ * dt;
    const double angular_diff = linear_diff * tan(steer_rad) / wheelbase_;

    wyaw_ = angular_diff / dt;

    if (fabs(angular_diff) < 1e-6)
    {
        const double direction = yaw_ + angular_diff * 0.5;

        x_   += linear_diff * cos(direction);
        y_   += linear_diff * sin(direction);
        yaw_ += angular_diff;
    } else {
        const double yaw_old = yaw_;
        const double radius = linear_diff/angular_diff;

        yaw_ += angular_diff;
        x_  +=  radius * (sin(yaw_) - sin(yaw_old));
        y_  += -radius * (cos(yaw_) - cos(yaw_old));
    }

    // Back wheel radius: wheelbase * cot(steer_rad) = wb * cos(s) / sin(s) = wb / tan(s)
    // https://www.ijser.org/researchpaper/optimizing-the-turning-radius-of-a-vehicle-using-symmetric.pdf

    // --- Old way to compute odometry ---

    // const double curvature_radius = wheelbase_ / cos(M_PI / 2.0 - steer_rad);
    // const double curvature_radius = wheelbase_ / sin(steer_rad);

    // if (fabs(curvature_radius) > MIN_CURVATURE_RADIUS)
    // {
    //     const double elapsed_distance = linear_diff;
    //     const double elapsed_angle = elapsed_distance / curvature_radius;
    //     const double x_curvature = curvature_radius * sin(elapsed_angle);
    //     const double y_curvature = curvature_radius * (cos(elapsed_angle) - 1.0);
    //     const double wheel_heading = yaw_ + steer_rad;
    //     y_ += x_curvature * sin(wheel_heading) + y_curvature * cos(wheel_heading);
    //     x_ += x_curvature * cos(wheel_heading) - y_curvature * sin(wheel_heading);
    //     yaw_ += elapsed_angle;
    // } else {

    //  }
}
