#include "drive_modes/drive_mode_base_class.h"
#include <cmath> 
DriveMode::DriveMode() {
//    ROS_INFO("DriveMode constructor");
}

void DriveMode::set_velocity_point(double x, double y) {
    this->x = x;
    this->y = y;
}

void DriveMode::apply(double& vx, double& vy, double& va) { }

void DriveMode::set_parameters(robotrainer_controllers::DriveModeParameters& 
dm_params) { }

double DriveMode::dot(double v1, double v2, double v3,
                      double w1, double w2, double w3) {
    return v1*w1 + v2*w2 + v3*w3;
}

double DriveMode::norm(double v1, double v2, double v3) {
    return std::sqrt(this->dot(v1, v2, v3, v1, v2, v3));
}
                                
void DriveMode::normalize(double& v1, double& v2, double& v3) {
    double n = this->norm(v1, v2, v3);
    v1 /= n;
    v2 /= n;
    v3 /= n;
}
                                
// Project v on u. v gets overwritten. u is assumed to be of unit size.
void DriveMode::project_onto_line(double& v1, double& v2, double& v3,
                                  double u1, double u2, double u3) {
    double v_dot_u = this->dot(v1, v2, v3, u1, u2, u3);
    v1 = u1 * v_dot_u;
    v2 = u2 * v_dot_u;
    v3 = u3 * v_dot_u;
}

// Project v on a plane defined by the normal u. v gets overwritten. u is
// assumed to be of unit size.
void DriveMode::project_onto_plane(double& v1, double& v2, double& v3,
                                   double u1, double u2, double u3) {
    double p1 = v1, p2 = v2, p3 = v3;
    this->project_onto_line(p1, p2, p3, u1, u2, u3);
    v1 -= p1;
    v2 -= p2;
    v3 -= p3;
}

void DriveMode::rotate(double angle, double& x, double& y) {
    x = x * std::cos(angle) - y * std::sin(angle);
    y = x * std::sin(angle) + y * std::cos(angle);
}
