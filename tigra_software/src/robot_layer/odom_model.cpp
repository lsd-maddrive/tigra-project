#include <robot_layer/odom_model.hpp>


void OdometryBicycleModel::updateState(double steer_rad, double speed_rps, double ts)
{
    if (prev_ts_ < 0)
    {
        prev_ts_ = ts;
        return;
    }

    const double dt = ts - prev_ts_;
    prev_ts_ = ts;

    const double linear = speed_rps * rps2mps_ * dt;
    const double angular = linear * tan(steer_rad) / wheelbase_;

    const double curvature_radius = wheelbase_ / cos(M_PI / 2.0 - steer_rad);

    if (fabs(curvature_radius) > MIN_CURVATURE_RADIUS)
    {
        const double elapsed_distance = linear;
        const double elapsed_angle = elapsed_distance / curvature_radius;
        const double x_curvature = curvature_radius * sin(elapsed_angle);
        const double y_curvature = curvature_radius * (cos(elapsed_angle) - 1.0);
        const double wheel_heading = yaw_ + steer_rad;
        y_ += x_curvature * sin(wheel_heading) + y_curvature * cos(wheel_heading);
        x_ += x_curvature * cos(wheel_heading) - y_curvature * sin(wheel_heading);
        yaw_ += elapsed_angle;
    }
}
