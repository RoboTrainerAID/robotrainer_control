#include "drive_modes/dead_reckon.h"

#include <cmath>

DeadReckonMode::DeadReckonMode() {
    this->pos_x = 0.0;
    this->pos_y = 0.0;
    this->pos_a = 0.0;

    this->msg_xi.x = 0;
    this->msg_xi.y = 0;
    this->msg_xi.z = 0;

    this->pub_xi = nh.advertise<geometry_msgs::Vector3>(
        "/platform/position", 100);


    ROS_INFO("DeadReckonMode constructor");
}

void DeadReckonMode::calculateTimeDiff() {
    static unsigned long lastNSec = ros::Time::now().toNSec();
    unsigned long now = ros::Time::now().toNSec();
    this->time_diff = (double)(now - lastNSec) * 1e-9;
    lastNSec = now;
}

void DeadReckonMode::keepTrack(double& vx, double& vy, double& va) {
    da = va * time_diff;

    // da/2 because I want arithmetic mean of beginning and the end.
    const double cos = std::cos(pos_a + da/2.0);
    const double sin = std::sin(pos_a + da/2.0);

    // dx, dy is the movement in global coordinate system. Here we apply the
    // rotation matrix.
    dx = (vx*cos - vy*sin) * time_diff;
    dy = (vx*sin + vy*cos) * time_diff;

    pos_x += dx;
    pos_y += dy;
    pos_a += da;
    while (pos_a < 0)        pos_a += 2 * M_PI;
    while (pos_a > 2 * M_PI) pos_a -= 2 * M_PI;

    // Optional
    publishOrientation();
}

void DeadReckonMode::publishOrientation() {
    msg_xi.x = pos_x;
    msg_xi.y = pos_y;
    msg_xi.z = pos_a;
    pub_xi.publish(msg_xi);
}

void DeadReckonMode::dr_apply(double& vx, double& vy, double& va) {
}

void DeadReckonMode::apply(double& vx, double& vy, double& va) {
    calculateTimeDiff();
    dr_apply(vx, vy, va);
    keepTrack(vx, vy, va);
}
