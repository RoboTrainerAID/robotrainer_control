/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "robotrainer_controllers/fts_GeomController.h"
#include "robotrainer_controllers/fts_wheelControllerBase.h"

#include <std_srvs/Trigger.h>

#include <dynamic_reconfigure/server.h>
#include <cob_omni_drive_controller/SteerCtrlConfig.h>

namespace robotrainer_controllers
{

class WheelController : public WheelControllerBase< OdomGeomController<hardware_interface::VelocityJointInterface, UndercarriageCtrl> >
{
public:
    virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh){
        ros::NodeHandle wheels_nh(root_nh, "wheel_controller");
        wheel_ctrl_nh_ = wheels_nh;

        if(!cob_omni_drive_controller::parseWheelParams(wheel_params_, wheel_ctrl_nh_) || !OdomGeomController::init(hw, wheel_params_)) return false;

        pos_ctrl_.init(wheel_params_, controller_nh);

        // Kinematics services
        service_kinematic_update_ = controller_nh.advertiseService("update_kinematics", &WheelController::updateWheelParamsCallback, this);

        return setup(root_nh, controller_nh);
    }
    virtual void update(const ros::Time& time, const ros::Duration& period){

        target_.state.setVelX(limitValue(twist_command_.linear.x, max_vel_trans_));
        target_.state.setVelY(limitValue(twist_command_.linear.y, max_vel_trans_));
        target_.state.dRotRobRadS = limitValue(twist_command_.angular.z, max_vel_rot_);
        target_.updated = true;
        target_.stamp = ros::Time::now();

        boost::mutex::scoped_try_lock lock_geom(mutex_geom_);
        if (lock_geom) {
            updateState();
            pos_ctrl_.try_configure(*geom_);

            updateCtrl(time, period);

            for (unsigned i=0; i<wheel_commands_.size(); i++){
                steer_joints_[i].setCommand(wheel_commands_[i].dVelGearSteerRadS);
                drive_joints_[i].setCommand(wheel_commands_[i].dVelGearDriveRadS);
            }
        }

    }

    bool updateWheelParamsCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {

        ROS_DEBUG("Called service to update kinematic configuration");
        boost::mutex::scoped_lock lock(mutex_geom_);
        for (unsigned i=0; i<wheel_commands_.size(); i++){
            steer_joints_[i].setCommand(0.0);
            drive_joints_[i].setCommand(0.0);
        }
        resp.success = true;
        resp.success &= cob_omni_drive_controller::parseWheelParams(wheel_params_, wheel_ctrl_nh_);
        resp.success &= OdomGeomController::update(wheel_params_);
        ROS_INFO("Kinematics sucessfully updated!");

        return true;
    }


    class PosCtrl {
    public:
        PosCtrl() : updated(false) {}
        void try_configure(UndercarriageCtrl &ctrl){
            boost::recursive_mutex::scoped_try_lock lock(mutex);
            if(lock && updated){
                ctrl.configure(pos_ctrl_params);
                updated = false;
            }
        }
        void init(const wheel_params_t &params, const ros::NodeHandle &nh){
            boost::recursive_mutex::scoped_lock lock(mutex);
            pos_ctrl_params.resize(params.size());
            reconfigure_server_axes_.clear();

            reconfigure_server_.reset( new dynamic_reconfigure::Server<cob_omni_drive_controller::SteerCtrlConfig>(mutex,ros::NodeHandle(nh, "default/steer_ctrl")));
            reconfigure_server_->setCallback(boost::bind(&PosCtrl::setForAll, this, _1, _2)); // this writes the default into pos_ctrl_params
            {
                cob_omni_drive_controller::SteerCtrlConfig config;
                copy(config, params.front().pos_ctrl);
                reconfigure_server_->setConfigDefault(config);
            }

            for(size_t i=0; i< pos_ctrl_params.size(); ++i) {
                boost::shared_ptr<dynamic_reconfigure::Server<cob_omni_drive_controller::SteerCtrlConfig> > dr(new dynamic_reconfigure::Server<cob_omni_drive_controller::SteerCtrlConfig>(mutex, ros::NodeHandle(nh, params[i].geom.steer_name)));
                cob_omni_drive_controller::SteerCtrlConfig config;
                copy(config, params[i].pos_ctrl);
                dr->setConfigDefault(config);
                dr->updateConfig(config);
                dr->setCallback(boost::bind(&PosCtrl::setForOne, this, i, _1, _2));  // this writes the joint-specific config into pos_ctrl_params
                reconfigure_server_axes_.push_back(dr);
            }
        }
    private:
        static void copy(PosCtrlParams &params, const cob_omni_drive_controller::SteerCtrlConfig &config){
            params.dSpring = config.spring;
            params.dDamp = config.damp;
            params.dVirtM = config.virt_mass;
            params.dDPhiMax = config.d_phi_max;
            params.dDDPhiMax = config.dd_phi_max;
        }
        static void copy(cob_omni_drive_controller::SteerCtrlConfig &config, const PosCtrlParams &params){
            config.spring = params.dSpring;
            config.damp = params.dDamp;
            config.virt_mass = params.dVirtM;
            config.d_phi_max = params.dDPhiMax;
            config.dd_phi_max = params.dDDPhiMax;
        }
        // these function don't get locked
        void setForAll(cob_omni_drive_controller::SteerCtrlConfig &config, uint32_t /*level*/) {
            ROS_DEBUG("configure all steers: s: %lf, d: %lf, m: %lf, v: %lf, a: %lf", config.spring, config.damp, config.virt_mass, config.d_phi_max, config.dd_phi_max);
            for(size_t i=0; i< pos_ctrl_params.size(); ++i) {
                copy(pos_ctrl_params[i], config);
                if(!reconfigure_server_axes_.empty()){
                    reconfigure_server_axes_[i]->setConfigDefault(config);
                    reconfigure_server_axes_[i]->updateConfig(config);
                }
            }
            updated = true;
        }
        void setForOne(size_t i, cob_omni_drive_controller::SteerCtrlConfig &config, uint32_t /*level*/) {
            ROS_INFO("configure steer %d: s: %lf, d: %lf, m: %lf, v: %lf, a: %lf", (int)i, config.spring, config.damp, config.virt_mass, config.d_phi_max, config.dd_phi_max);
            copy(pos_ctrl_params[i], config);
            updated = true;
        }
        std::vector<PosCtrlParams> pos_ctrl_params;
        boost::recursive_mutex mutex; // dynamic_reconfigure::Server calls the callback from the setCallback function
        bool updated;
        boost::scoped_ptr< dynamic_reconfigure::Server<cob_omni_drive_controller::SteerCtrlConfig> > reconfigure_server_;
        std::vector<boost::shared_ptr< dynamic_reconfigure::Server<cob_omni_drive_controller::SteerCtrlConfig> > > reconfigure_server_axes_; // TODO: use unique_ptr
    } pos_ctrl_;

private:
    ros::NodeHandle wheel_ctrl_nh_;
    ros::ServiceServer service_kinematic_update_;

    wheel_params_t wheel_params_;

    boost::mutex mutex_geom_;
};

}

PLUGINLIB_EXPORT_CLASS(robotrainer_controllers::WheelController, controller_interface::ControllerBase)
