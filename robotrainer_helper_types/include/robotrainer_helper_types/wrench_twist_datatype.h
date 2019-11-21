#ifndef ROBOTRAINER_HELPER_TYPES_WRENCH_TWIST_DATATYPE_H
#define ROBOTRAINER_HELPER_TYPES_WRENCH_TWIST_DATATYPE_H

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
namespace robotrainer_helper_types{
	
typedef struct wrenchStamped_twist_{
	geometry_msgs::WrenchStamped wrench_stamped_;
	geometry_msgs::Twist twist_;
} wrenchStamped_twist;
	
typedef struct wrench_twist_{
	geometry_msgs::Wrench wrench_;
	geometry_msgs::Twist twist_;
} wrench_twist;

}

#endif
