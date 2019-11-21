#include <robotrainer_parameters/shared_params.h>

namespace robotrainer_parameters
{

/** 
 * \brief Constructor
 * 
 * Sets up dynamic reconfigure and rosparam_handler
 */
SharedParams::SharedParams()
: params_{ros::NodeHandle("/robotrainer")}, dynamic_reconfigure_server_{ros::NodeHandle("/robotrainer")}
{
    params_.fromParamServer();
    dynamic_reconfigure_server_.setCallback(boost::bind(&SharedParams::reconfigureRequest, this, _1, _2));
}

/**
 * \brief Callback function for dynamic reconfigure.
 */
void SharedParams::reconfigureRequest(SharedParamsConfig& config, uint32_t level)
{   
    params_.fromConfig(config);
};

robotrainer_parameters::SharedParamsParameters* SharedParams::getParamsPtr()
{
    return &params_;
}

}
