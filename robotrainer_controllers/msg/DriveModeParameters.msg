# Many classes use field names from this message. You may add new fields without
# any consequences, but do not change any existing fields.

# UndercarriageDriveMode accepts a velocity and based on that, finds a suitable
# velocity that is in line with the current drive mode. These input/output 
# velocities are not expressed at the platform center, but at this point
# (expressed in platform coordinate system). Some modes ignore it, though.
# Example:
# In omnidirectional mode, requesting the velocity with components [0.0 0.0
# 0.2] would result in the platform rotating around itself, without any
# translation. But if this point were set to [1.0 0.0], the same request would
# make the robot revolve around a point 1m away from the origin of the platform
# coordinate system.
float64 velocity_point_x
float64 velocity_point_y

ICR icr

VirtualAxle virtual_axle

# Relevant for Ackermann mode. The minimum turning radius. Measured from axle
# center.
float64 r_min

float64 p

CastorParams castor

# To be used in "Ax+b mode"
A_and_b a_and_b
