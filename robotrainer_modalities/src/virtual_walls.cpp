#include <pluginlib/class_list_macros.h>
#include <robotrainer_modalities/modality_base.h>
#include <robotrainer_modalities/virtual_walls.h>

PLUGINLIB_EXPORT_CLASS(robotrainer_modalities::VirtualWalls<geometry_msgs::Twist>, filters::FilterBase<geometry_msgs::Twist>)
PLUGINLIB_EXPORT_CLASS(robotrainer_modalities::VirtualWalls<geometry_msgs::Twist>, robotrainer_modalities::ModalityBase<geometry_msgs::Twist>)
