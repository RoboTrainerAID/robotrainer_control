#include <pluginlib/class_list_macros.h>
#include <robotrainer_modalities/modality_base.h>
#include <robotrainer_modalities/virtual_forces.h>

PLUGINLIB_EXPORT_CLASS(robotrainer_modalities::VirtualForces<geometry_msgs::Twist>, filters::FilterBase<geometry_msgs::Twist>)
PLUGINLIB_EXPORT_CLASS(robotrainer_modalities::VirtualForces<geometry_msgs::Twist>, robotrainer_modalities::ModalityBase<geometry_msgs::Twist>)
