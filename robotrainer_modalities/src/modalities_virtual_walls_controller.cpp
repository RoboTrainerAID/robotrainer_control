#include "robotrainer_modalities/modalities_virtual_walls_controller.h"

#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>


namespace robotrainer_modalities{
	
template <typename T> ModalitiesVirtualWallsController<T>::ModalitiesVirtualWallsController() : params_{ros::NodeHandle("/modalities/modalitiesVirtualWallsController")}, dynamic_reconfigure_server_{ros::NodeHandle("/modalities/modalitiesVirtualWallsController")}
{
					//initialize the defaultvalues s
					params_.fromParamServer();
					
					// Dynamic reconfigure
					dynamic_reconfigure_server_.setCallback(boost::bind(&ModalitiesVirtualWallsController<T>::reconfigureRequest, this, _1, _2));
	
					//initialize virtualForces without Filterchain
			 pluginlib::ClassLoader<ModalityBase<geometry_msgs::Twist>> vf_loader_ptr_("robotrainer_modalities", "robotrainer_modalities::ModalityBase<geometry_msgs::Twist>");
				try{
					vf_modalitie_ptr_ = vf_loader_ptr_.createInstance("robotrainer_modalities/VirtualWalls");
					ROS_INFO_ONCE("[modalities_virtual_walls_controller.cpp] VirtualWallsModalitie loaded");
					this->modalitie_loaded_ = true;
				}
				catch(pluginlib::PluginlibException& e){
					ROS_ERROR_STREAM("Modality plugin failed to load:" << e.what());
					this->modalitie_loaded_ = false;
				}
				this->modalitie_configured_ = false;
				
				test_pub_ = nh_.advertise<geometry_msgs::Vector3>("modalities/modalitiesVirtualWallsController/test", 1000);
				HF_HV_Energy_pub_ = nh_.advertise<std_msgs::Float64>("modalities/modalitiesVirtualWallsController/hf_vh_energy", 1000);
				VF_MV_Energy_pub_ = nh_.advertise<std_msgs::Float64>("modalities/modalitiesVirtualWallsController/hf_mh_energy", 1000);
}


template <typename T> bool ModalitiesVirtualWallsController<T>::update(const T& data_in, T& data_out)
{
// 	ROS_INFO("[modalities_virtual_walls_controller.cpp] update");
	if(this->modalitie_configured_){
		data_out = data_in;
		
		vf_modalitie_ptr_->update(data_in.twist_, data_out.twist_);

		if(use_modalities_virtual_walls_controller_){
// 		ROS_INFO("[modalities_virtual_walls_controller.cpp] use = true");
		// create human force modalities velocity msg
		hf_mv_msg_.wrench_ = data_in.wrench_;
		hf_mv_msg_.twist_ = data_out.twist_;
		
		//create virtual force modalities velocity msg 
		vf_mv_msg_.wrench_.force = tf2::toMsg(vf_modalitie_ptr_->getLastscaledForce());
		vf_mv_msg_.wrench_.torque = tf2::toMsg(null_vec_);
		vf_mv_msg_.twist_= data_out.twist_;

		//calculateIntegral
		integral_HF_MV_ptr_->calculateIntegral(hf_mv_msg_, ros::Time::now());
		integral_VF_MV_ptr_->calculateIntegral(vf_mv_msg_, ros::Time::now());
		
		//TODO
		
		tf2::Vector3 human_input_force;
		tf2::fromMsg(hf_mv_msg_.wrench_.force, human_input_force);
		tf2::Vector3 human_twist_lin;
		tf2::fromMsg(data_in.twist_.linear, human_twist_lin);
		tf2::Vector3 virtual_twist_lin;
		tf2::fromMsg(data_out.twist_.linear, virtual_twist_lin);
		virtual_twist_lin = virtual_twist_lin - human_twist_lin;
		//outside of virtual force area
		//Multidim allows pass through walls
// 		if(virtual_twist_lin.length() <= 0.0){
// 			virtual_twist_lin = human_twist_lin;
// 		}else if(checkHumanLetRobotGo(human_input_force)) {
// 			vel_limit_factor_vec_ = calcVelLimitationFactorPerDim(human_twist_lin, virtual_twist_lin);
// 			virtual_twist_lin = human_twist_lin + virtual_twist_lin * vel_limit_factor_vec_;
// 		}else if(humanEndangeredVelocity()){
// 			vel_limit_factor_vec_ = calcVelLimitationFactorPerDim(human_twist_lin, virtual_twist_lin);
// 			virtual_twist_lin = human_twist_lin + virtual_twist_lin * vel_limit_factor_vec_;
// 		}else {
// 			vel_limit_factor_vec_ = scaleVelFactorBackVec(calcVelLimitationFactorPerDim(human_twist_lin, virtual_twist_lin));
// 			virtual_twist_lin = human_twist_lin + virtual_twist_lin * vel_limit_factor_vec_;
// 		}
		
		if(virtual_twist_lin.length() <= 0.0){
			virtual_twist_lin = human_twist_lin;
		}else if(checkHumanLetRobotGo(human_input_force) || humanEndangeredVelocity()){
			vel_limit_factor_ = calcVelLimitationFactor(human_twist_lin, virtual_twist_lin);
			virtual_twist_lin = human_twist_lin + virtual_twist_lin * vel_limit_factor_;
		} else{
			
			vel_limit_factor_ = scaleVelFactorBack(calcVelLimitationFactor(human_twist_lin, virtual_twist_lin));
			double walls_limit_vel_factor = vf_modalitie_ptr_->getLastLimitVelocityFactor();
// 			if(test_){
// 				double walls_limit_vel_factor = vf_modalitie_ptr_->getLastLimitVelocityFactor();
// 				virtual_twist_lin = human_twist_lin * walls_limit_vel_factor + virtual_twist_lin * vel_limit_factor_;
// 			}else{
				virtual_twist_lin = human_twist_lin + virtual_twist_lin * vel_limit_factor_;
// 			}
		}
		
		data_out.twist_.linear.x = virtual_twist_lin.getX();
		data_out.twist_.linear.y = virtual_twist_lin.getY();
		data_out.twist_.linear.z = virtual_twist_lin.getZ();
		
// 		} else {
// 			vel_limit_factor_ = scaleVelFactorBack(1.0);
// 			virtual_twist_lin = human_twist_lin + virtual_twist_lin * vel_limit_factor_;
// 		}
		data_out.twist_.linear.x = virtual_twist_lin.getX();
		data_out.twist_.linear.y = virtual_twist_lin.getY();
		data_out.twist_.linear.z = virtual_twist_lin.getZ();
		}
	}else{
		ROS_WARN_ONCE("[modalities_virtual_walls_controller.cpp] modalities not configured");
	}
		
}


template <typename T> bool  ModalitiesVirtualWallsController<T>::configure()
{
	this->modalitie_configured_ = vf_modalitie_ptr_->configure();
	ROS_INFO_COND(this->modalitie_configured_, "[modalities_virtual_walls_controller.cpp]vf_modality activated");
	ROS_INFO_COND(!this->modalitie_configured_,"[modalities_virtual_walls_controller.cpp]vf_modalitie activation FAILED");
	return this->modalitie_configured_;
}

template <typename T> tf2::Vector3 ModalitiesVirtualWallsController<T>::getLastVelocity()
{
	return vf_modalitie_ptr_->getLastVelocity();
}
			
template <typename T> tf2::Vector3 ModalitiesVirtualWallsController<T>::getLastscaledForce()
{ 
	return vf_modalitie_ptr_->getLastscaledForce();
}

template <typename T> bool ModalitiesVirtualWallsController<T>::checkHumanLetRobotGo(const tf2::Vector3& human_input_force)
{
		if((human_input_force.length() < 1.0)) {
			return true;
		}
		return false;
}


template <typename T> bool ModalitiesVirtualWallsController<T>::humanEndangeredVelocity()
{
			//get Integralvalues
			double hf_mv_val = integral_HF_MV_ptr_->getIntegralValue();
			double vf_mv_val = integral_VF_MV_ptr_->getIntegralValue();
// 			ROS_INFO("[modalities_virtual_walls_controller.cpp] hf_mv_val: [%.2f]|| vf_mv_val: [%.2f]", hf_mv_val, vf_mv_val);
			//check for endangering case
			if(	hf_mv_val < 0.0 && vf_mv_val > 0.0){
				tf2::Vector3 resulting_velocity_vector;
				tf2::fromMsg(vf_mv_msg_.twist_.linear, resulting_velocity_vector);
				
				double angle = (tf2::tf2Angle(resulting_velocity_vector, minus_unit_x_vec_ )) * rad_to_deg_const_;
// 				ROS_INFO("[modalities_virtual_walls_controller.cpp] ANGLE: [%.2f]", angle);
				//Maybe add && (last_scaled_vf.getX() < 0.0) because of jump at pi
				if((angle <= max_abs_angle_)){
					return true;
				}
			}
			return false;
}


template <typename T> geometry_msgs::Twist ModalitiesVirtualWallsController<T>::limitModalitiesVelocity(geometry_msgs::Twist input_twist)
{
		tf2::Vector3 last_scaled_vf;
		tf2::Vector3 last_hf;
		
		tf2::fromMsg(vf_mv_msg_.wrench_.force, last_scaled_vf);
		tf2::fromMsg(hf_mv_msg_.wrench_.force, last_hf);
	
		double last_scaled_vf_length = last_scaled_vf.length();
		double  last_hf_length = last_hf.length();
		
		
		double limitation_factor = (last_scaled_vf_length > last_hf_length) ? (last_hf_length/last_scaled_vf_length) : 1.0;
		
		ROS_INFO("[modalities_virtual_walls_controller.cpp] LimitFactor:[%.2f]", limitation_factor);
		
		tf2::Vector3 input_vel;
		tf2::Vector3 vel_after_modalities;
		tf2::fromMsg(input_twist.linear, input_vel);
		tf2::fromMsg(vf_mv_msg_.twist_.linear, vel_after_modalities);
		
		tf2::Vector3 adapted_modalites_vel = (vel_after_modalities - input_vel) * (limitation_factor);
		geometry_msgs::Twist out_twist;
		out_twist.linear = tf2::toMsg(adapted_modalites_vel);
		out_twist.angular = tf2::toMsg(null_vec_);
		
		out_twist.linear.x += input_twist.linear.x;
		out_twist.linear.y += input_twist.linear.y;
		out_twist.linear.z += input_twist.linear.z;
		out_twist.angular.x += input_twist.angular.x;
		out_twist.angular.y += input_twist.angular.y;
		out_twist.angular.z += input_twist.angular.z;
		
		return out_twist;
	
}

template <typename T> double ModalitiesVirtualWallsController<T>::calcVelLimitationFactor(const tf2::Vector3& human_vel_lin, const tf2::Vector3& modalities_vel_lin)
{
	return (human_vel_lin.length() <= modalities_vel_lin.length()) ? (human_vel_lin.length() / modalities_vel_lin.length() ) : 1.0;
}

template <typename T> tf2::Vector3 ModalitiesVirtualWallsController<T>::calcVelLimitationFactorPerDim(const tf2::Vector3& human_vel_lin, const tf2::Vector3& modalities_vel_lin)
{
	double x = (std::abs(human_vel_lin.getX()) <= std::abs(modalities_vel_lin.getX())) ? (std::abs(human_vel_lin.getX()) / std::abs(modalities_vel_lin.getX()) ) : 1.0;
	double y = (std::abs(human_vel_lin.getY()) <= std::abs(modalities_vel_lin.getY())) ? (std::abs(human_vel_lin.getY()) / std::abs(modalities_vel_lin.getY()) ) : 1.0;
	tf2::Vector3 vel_limit_factor_vec(x, y, 0.0);
	return vel_limit_factor_vec;
}

template <typename T> double ModalitiesVirtualWallsController<T>::scaleVelFactorBack(const double& vel_limit_factor_goal)
{
	double scaled_vel_lim_fac = vel_limit_factor_ * (1 - forgetting_factor_) + vel_limit_factor_goal * forgetting_factor_;
	//lim from smoothing function -> vel_limit_factor_goal but "never" reaches it. This ensures that vel_limit_factor_goal is reached
	if(std::abs(vel_limit_factor_goal - scaled_vel_lim_fac) <= eps_){
		scaled_vel_lim_fac = vel_limit_factor_goal;
	}
	return (scaled_vel_lim_fac <= 1.0) ? scaled_vel_lim_fac : 1.0;
	
}

template <typename T> tf2::Vector3 ModalitiesVirtualWallsController<T>::scaleVelFactorBackVec(const tf2::Vector3& vel_limit_factor_goal)
{
	tf2::Vector3 scaled_vel_lim_fac_vec = vel_limit_factor_vec_ * (1 - forgetting_factor_) + vel_limit_factor_goal * forgetting_factor_;
	//lim from smoothing function -> vel_limit_factor_goal but "never" reaches it. This ensures that vel_limit_factor_goal is reached
	if(std::abs(vel_limit_factor_goal.getX() - scaled_vel_lim_fac_vec.getX()) <= eps_){
		scaled_vel_lim_fac_vec.setX(vel_limit_factor_goal.getX());
	}
	if(std::abs(vel_limit_factor_goal.getY() - scaled_vel_lim_fac_vec.getY()) <= eps_){
		scaled_vel_lim_fac_vec.setY(vel_limit_factor_goal.getY());
	}
	if(scaled_vel_lim_fac_vec.getX() > 1.0)
	{
		scaled_vel_lim_fac_vec.setX((1.0));
	}
	if(scaled_vel_lim_fac_vec.getY() > 1.0)
	{
		scaled_vel_lim_fac_vec.setY((1.0));
	}
	return scaled_vel_lim_fac_vec;
}

template <typename T> void ModalitiesVirtualWallsController<T>::reconfigureRequest(const robotrainer_modalities::ModalitiesVirtualWallsControllerConfig& config, uint32_t level)
{
	params_.fromConfig(config);
	
	use_modalities_virtual_walls_controller_ = params_.use_modalities_virtual_walls_controller;
// 	limit_modalities_vel_norm_ = params_.limit_modalities_vel_norm;
	integral_HF_MV_window_size_ = params_.integral_HF_MV_window_size;
	integral_VF_MV_window_size_ = params_.integral_VF_MV_window_size;
	max_abs_angle_ = params_.max_abs_angle;
	forgetting_factor_ = params_.forgetting_factor;
	eps_ = params_.eps;
// 	test_ = params_.test;
	
	integral_HF_MV_ptr_.reset(new robotrainer_helper_types::SlidingIntegral{integral_HF_MV_window_size_});
	integral_VF_MV_ptr_.reset(new robotrainer_helper_types::SlidingIntegral{integral_VF_MV_window_size_});
}


}//namespace

PLUGINLIB_EXPORT_CLASS(robotrainer_modalities::ModalitiesVirtualWallsController<robotrainer_helper_types::wrench_twist>, robotrainer_modalities::ModalitiesControllerBase<robotrainer_helper_types::wrench_twist>)

