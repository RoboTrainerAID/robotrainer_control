#ifndef ROBOTRAINER_PARAMETERS_SHARED_PARMS_H
#define ROBOTRAINER_PARAMETERS_SHARED_PARMS_H

#include <ros/ros.h>
#include <robotrainer_parameters/SharedParamsParameters.h>
#include <robotrainer_parameters/SharedParamsConfig.h>
#include <dynamic_reconfigure/server.h>

namespace robotrainer_parameters
{
    
/**
 * \brief
 */
class SharedParams
{
public:
    robotrainer_parameters::SharedParamsParameters* getParamsPtr();
    SharedParams();
private:
    // Objects for param_handler and dynamic_reconfigure
    robotrainer_parameters::SharedParamsParameters params_;
    void reconfigureRequest(robotrainer_parameters::SharedParamsConfig& config, uint32_t level);
    dynamic_reconfigure::Server<robotrainer_parameters::SharedParamsConfig> dynamic_reconfigure_server_;
};

};

#endif
