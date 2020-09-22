#include "drive_modes/pivot_with_moving_focus.h"

#include <cmath>
#include <limits>

#define ZERO(X) (-0.0000001 < (X) && (X) < 0.0000001)

void unwrap_radians(double& rd) {
    while (rd < 0.0) rd += 2 * M_PI;
    while (rd > 2 * M_PI) rd -= 2 * M_PI;
}

PivotModeWithMovingFocus::PivotModeWithMovingFocus() {
    pub_xi = nh.advertise<geometry_msgs::Vector3>("/platform/position", 1000);
    pub_beta = nh.advertise<std_msgs::Float64>("/castor/beta", 1000);

    ROS_INFO("PivotModeWithMovingFocus constructor");
    this->C_beta = 0.0;
    this->param = .001; // in case they don't set_parameters
    this->of.open("/tmp/fifo");

    this->msg_xi.x = 0;
    this->msg_xi.y = 0;
    this->msg_xi.z = 0;
    this->msg_beta.data = C_beta;
}

PivotModeWithMovingFocus::~PivotModeWithMovingFocus() {
    this->of.close();
}

double PivotModeWithMovingFocus::function(double a) {
    return - ((fx - hx) * std::cos(a) + d) / std::sin(a);
}

double PivotModeWithMovingFocus::solver(double beg, double end, double target) {
    // Function is increasing for positive fx (> d).
    double k = fx > 0.0 ? 1.0 : -1.0; 

    // Start the binary search at the middle. 
    double solution = beg + (end - beg) / 2.0;

    for (double inc = (end - beg) / 2.0; inc > 0.0000001; inc /= 2.0) {
        if (k * function(solution) < k * target) {
            solution += inc;
        }
        else {
            solution -= inc;
        }
    }
    return solution;
}

double PivotModeWithMovingFocus::input_rotation(double vy, double va) {
    double len = std::sqrt(1.0 + fx * fx);
    double u_vy = -1.0 / len;
    double u_va = fx / len;
    return u_va * (vy * u_vy + va * u_va);
}

double PivotModeWithMovingFocus::find_angle_should(double vx, double rot) {
    if (rot > 0.0) {
        return solver(0.0, M_PI, vx / rot);
    }
    else if (rot < 0.0) {
        return solver(M_PI, 2.0 * M_PI, vx / rot);
    }
    else {
        if (vx > 0.0) {
            return 0.0;
        }
        else if (vx < 0.0) {
            return M_PI;
        }
        else {
            return C_beta;
        }
    }
}

void PivotModeWithMovingFocus::apply(double& vx, double& vy, double& va) {
    // Do nothing if input velocity is zero.
    if (ZERO(vx) && ZERO(vy) && ZERO(va)) {
        return;
    }

    this->of << "BEFORE APPLY\n"
             << "C_beta: " << C_beta * 180 / M_PI
             << " vx: " << vx
             << " vy: " << vy
             << " va: " << va
             << "\n";

//    // Castor on the middle axis. Therefore merge vy and va.
//    double in_rot = input_rotation(vy, va);
//    this->of << "in_rot: " << in_rot;
//
//    double C_beta_dif = in_rot + 
//    (vx * std::sin(C_beta) - in_rot * (-fx) * std::cos(C_beta)) / d;

    double C_beta_dif = 
    (std::cos(M_PI+C_beta)*vx+std::sin(M_PI+C_beta)*vy+(d+fx)*std::sin(C_beta))
    / (-d);


//    if (std::abs(C_beta_dif) < param) C_beta_dif = 0.0;
//    else {
//        if (C_beta_dif > 0.0) C_beta_dif -= param;
//        else C_beta_dif += param;
//    }


    // C_beta -= C_beta_dif;
    C_beta += (C_beta_dif * param);
    unwrap_radians(C_beta);

/*
    double angle_should = find_angle_should(vx, in_rot);

    this->of << "angle_should: " << angle_should * 180 / M_PI << "\n";

    double middle = weighted_middle(C_beta, angle_should);

    // The slower the platform is, the slower the focus should move.
    //C_beta += (middle - C_beta) * norm(vx, fx * in_rot, in_rot);
    C_beta += (middle - C_beta);
*/
    
    double fy = function(C_beta);
    PivotMode::set_focus(fx, fy);

    PivotMode::apply(vx, vy, va);

    this->of << "AFTER APPLY\n"
             << "C_beta: " << C_beta * 180 / M_PI
             << " vx: " << vx
             << " vy: " << vy
             << " va: " << va
             << "\n";
    this->of.flush();

    msg_xi.x += vx / 200;
    msg_xi.y += vy / 200;
    msg_xi.z += va / 200;
    pub_xi.publish(msg_xi);
    msg_beta.data = C_beta;
    pub_beta.publish(msg_beta);
}

void PivotModeWithMovingFocus::set_parameters(
robotrainer_controllers::DriveModeParameters& dm_params) {
    set_velocity_point(dm_params.velocity_point_x, dm_params.velocity_point_y);

    //TODO doc & rossrv
    this->d = 0.1;

    this->fx = dm_params.x;

    this->C_beta = dm_params.a;

    this->hx = dm_params.y;

    this->param = dm_params.r_min;
    if (this->param < 0.0) this->param = 0.0;
    //if (this->param > 1.0) this->param = 1.0;
}


double PivotModeWithMovingFocus::weighted_middle(double angle1, double angle2) {
    // Make sure the angles are in [0,2pi).
    // you could use unwrap_radians
    while (angle1 < 0.0) angle1 += 2 * M_PI;
    while (angle2 < 0.0) angle2 += 2 * M_PI;
    while (angle1 > 2 * M_PI) angle1 -= 2 * M_PI;
    while (angle2 > 2 * M_PI) angle2 -= 2 * M_PI;

    // Order the angles. Keep in mind which is which.
    bool min_is_angle1;
    double min, max;
    if (angle1 < angle2) {
        min = angle1;
        max = angle2;
        min_is_angle1 = true;
    }
    else {
        min = angle2;
        max = angle1;
        min_is_angle1 = false;
    }

    // Find the closer middle point.
    double ret;

    if (max - min > M_PI) max = -max;

    if (min_is_angle1) ret = param * min + (1.0 - param) * max;
    else               ret = param * max + (1.0 - param) * min;

    if (max - min > M_PI) ret = -ret;

    return ret;
}
