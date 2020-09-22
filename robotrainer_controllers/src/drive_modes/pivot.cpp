#include "drive_modes/pivot.h"

#include <cmath>

#define INF std::numeric_limits<double>::infinity()

PivotMode::PivotMode() {
    ROS_INFO("PivotMode constructor");
}

void PivotMode::apply(double& vx, double& vy, double& va) {
    project_onto_line(vx, vy, va, nx, ny, na);
}

void PivotMode::set_focus(double focus_x, double focus_y) {
    if (std::abs(focus_x) == INF) {
        this->nx = 0.0;
        this->ny = 1.0;
        this->na = 0.0;
    }
    else if (std::abs(focus_y) == INF) {
        this->nx = 1.0;
        this->ny = 0.0;
        this->na = 0.0;
    }
    else {
        this->nx = focus_y - this->y;
        this->ny = this->x - focus_x;
        this->na = 1.0;
        normalize(this->nx, this->ny, this->na);
    }
}

void PivotMode::set_parameters(robotrainer_controllers::DriveModeParameters&
dm_params) {
    set_velocity_point(dm_params.velocity_point_x, dm_params.velocity_point_y);
    set_focus(dm_params.icr.x, dm_params.icr.y);
}
