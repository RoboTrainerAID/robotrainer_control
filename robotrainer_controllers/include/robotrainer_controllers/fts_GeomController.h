#ifndef H_GEOM_CONTROLLER_IMPL
#define H_GEOM_CONTROLLER_IMPL
#include <hardware_interface/force_torque_sensor_interface.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/multi_interface_controller.h>
#include <cob_omni_drive_controller/UndercarriageCtrlGeomROS.h>
#include <boost/scoped_ptr.hpp>

namespace robotrainer_controllers
{

template<typename HandleType, typename Controller> class GeomControllerBase {
protected:
    std::vector<HandleType> steer_joints_;
    std::vector<HandleType> drive_joints_;
    boost::scoped_ptr<Controller> geom_;
    std::vector<WheelState> wheel_states_;

public:

    void updateState(){

        for (unsigned i=0; i<wheel_states_.size(); i++){
            wheel_states_[i].dAngGearSteerRad = steer_joints_[i].getPosition();
            wheel_states_[i].dVelGearSteerRadS = steer_joints_[i].getVelocity();
            wheel_states_[i].dVelGearDriveRadS = drive_joints_[i].getVelocity();
        }
        geom_->updateWheelStates(wheel_states_);
    }

//     void writeCommand(){
//
//     }
protected:
    bool setup(const std::vector<typename Controller::WheelParams> &wheel_params){
        if (wheel_params.size() < 3){
            ROS_ERROR("At least three wheel are needed.");
            return false;
        }
        wheel_states_.resize(wheel_params.size());
        ROS_DEBUG("Creating new Geometry Controller");
        geom_.reset(new Controller(wheel_params));
        return true;

    }
};

template<typename Controller> class GeomController:
    public GeomControllerBase<typename hardware_interface::VelocityJointInterface::ResourceHandleType, Controller>,
        public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface, hardware_interface::ForceTorqueSensorInterface> {
public:
    typedef std::vector<typename Controller::WheelParams> wheel_params_t;
    // TODO: This function can be commented out?
//     bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& controller_nh){
//         std::vector<typename Controller::WheelParams> wheel_params;
//         if(!parseWheelParams(wheel_params, controller_nh)) return false;
//         return init(hw, wheel_params);
//     }
    bool init(hardware_interface::VelocityJointInterface* hw, const wheel_params_t & wheel_params){
        if(!this->setup(wheel_params)) return false;
        try{
            // TODO: Check if there are already handles there and than skip this step
            for (unsigned i=0; i<wheel_params.size(); i++){
                this->steer_joints_.push_back(hw->getHandle(wheel_params[i].geom.steer_name));
                this->drive_joints_.push_back(hw->getHandle(wheel_params[i].geom.drive_name));
            }
        }
        catch(const std::exception &e){
            ROS_ERROR_STREAM("Error while attaching handles: " << e.what());
            return false;
        }
        return true;
    }

    bool update(const wheel_params_t & wheel_params) {
        if(!this->setup(wheel_params)) return false;
        return true;
    }
};

}

#endif
