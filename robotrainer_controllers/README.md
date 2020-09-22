# Drive Mode Module

Here is a short explanation of what each file related to the drive mode module
does.  The files are not in alphabetical order, in order to give them a
semantic order.

The files are commented, so for longer explanations, you can always look into
those.

#### srv/ucdm_cmd.srv:

The service `/base/undercarriage_drive_mode` is defined here. If the service
fails, i.e. invalid parameters, the return value of the `rosservice` process
indicates this. What is returns is not important, not really of any use. But we
can still say that, for the most part it echoes the input parameters.

This has exceptions though, for instance the differential mode does not send the
virtual axle, but instead the unit normal vector of the admissible drive command
plane. It was useful for development, but you can ignore it or change/delete
the returned message as you wish.

Only possible use of these return values (I doubt it will be needed) is for
querying current parameters. (This is done by sending a message to the service
with mode=0.)

## msg/:

#### DriveModeParameters.msg

The data type that the drive mode service expects. All other messages are part
of this message.

#### A_and_b.msg

3-by-3 matrix A and 3D vector b.

#### CastorParams.msg

To be used to define castor wheels.

#### ICR.msg

Instantaneous center of rotation. (x,y) in platform coordinates. Used by pivot
mode.

#### VirtualAxle.msg

Definition of a virtual axle. (x,y) axle center in platform coordinates. a the
angle between platform x-axis and the axle normal, in radians. Used by
differential and Ackermann modes.

## src/ and include/

If there is a file `src/maybeWithDir/abcd.cpp`, then
`include/maybeWithDir/abcd.cpp` exists as well. Few exceptions:

* drive_modes.h: This file includes all other include files. So it is only under
  `include/`, not under `src/`.
* ackermann_generalized.cpp: An unused implementation. Only under `src/`.

#### undercarriage_drive_mode_service.[cpp|h]

* Only drive-mode related file that is not under `[src|include]/drive_modes/`.
* Defines the ROS service.
* __Defines which number corresponds to which drive mode, like "2 is
  DifferentialMode "__
* Only interface between drive-mode module and the rest of the system.

All other files under `[src|include]/drive_modes/` are as follows: (with .cpp or 
.h extensions)

#### drive_mode_base_class

* Meant to be a base class for all other classes in the module.
* __Child classes often overwrite these methods:__ `apply` and `set_parameters`
* Defines some linear algebra functions.

#### omnidirectional

Omnidirectional mode, or practically, "no mode" because it does not alter the
drive command. Come back to this mode after using other modes in order to
restore the platform's functionality.

#### differential

* Emulates a virtual axle. The classic differential drive.

#### ackermann

* Like the differential mode, but now there is a minimum turning radius. This 
  way, the platform behaves like a car.
* Unlike the differential mode, in that only virtual axles that are parallel to
  the y-axis centered on the x-axis are allowed. Because it was easier to 
  implement. If a diagonal axis is needed, you could apply a linear transform
  on the drive command before feeding it into this mode, and then transform
  back.

#### ackermann_smoother

Like the Ackermann mode (previous), but "smoother". Instead of cutting off the
excess rotation in the steering wheels, all rotations are scaled. So there is
no need for a cut-off. Practically this means that the steering wheels will
not jump from left limit to the right limit. But this also makes the steering
wheels to respond less (because of the scale-down).

#### castor

This was supposed to be an emulation of a platform with two ideal fixed wheels
at the front and one non-ideal castor wheel in the middle at the back (pretty
much under the force-torque sensor). 

This does not work. (With right parameters and the right sequence of movements,
it sometimes feels like it is working.)

Many approaches were taken. Those are captured in the commented out `helper()`
methods.

Unlike other modes, castor mode sends messages about the platform state and 
about the virtual castor wheel. This was for visualization.

#### pivot_with_moving_focus

This was the first attempt at emulating a non-ideal castor wheel. Not only it 
does not work, it is also more complex than the following attempts.

#### line2dof

This was just a foray into what is possible. If you bring a shopping cart (with
four castor wheels) to a tram track and stick one wheel into the rail, you get
this behaviour. So it is 2 DOF, but a specified point on the platform must
remain on a line defined in the global coordinate system.

The current orientation the platform is needed for this to work properly. As it
is, it works, but it uses dead reckoning. So the error builds up pretty quickly
and the line cannot be followed properly. But if you introduce more precise
odometry, this is usable.

#### dead_reckon

line2dof needed this (previous mode). It keeps a running sum of the drive 
commands sent. Poor man's odometry. __Meant to be a base class.__

#### differential_with_matrices

This is another take at the differential drive mode. It works as it should, but
it is conceptually unsound. Still, if you try it with parameters in the [-1,1]
range, you might not notice a difference between this differential mode and the
other differential mode.

#### omni_with_max_accel

This was also an experiment. I wanted to see what happens when I set a maximum
acceleration. It does not work properly, because I observed what I wanted to 
observe. So I did not develop it further.

It works for speeding up, but when slowing down (more precisely, when the user
releases the handle), the force-torque controller calculates zero as the drive
command, and for zero drive commands, update() is not called...  Things get
complicated. Still, it feels like pushing a very heavy stone on ice.

#### pivot

This fixes the instantaneous center of rotation at a given point. So you have a 
carousel. You can also set the instantaneous center of rotation very, very far
away (e.g. a million meters) to constrain the platform onto a straight line.

#### synchro

Platform behaves like a synchro-drive robot: No rotation.

#### AxPLUSb

This mode allows you to modify the drive command c = [dx/dt dy/dy dθ/dt]^T 
using linear algebra: c_new = Ac+b. So the most generic mode, in a way.

Examples:
* Synchro-drive: Set A=diag(1,1,0)
* "Inverted controls": Set A=diag(1,-1,1).
* Emulate an artificial force field: Set A=0 and b≠0. 
