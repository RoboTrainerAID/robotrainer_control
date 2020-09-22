/*

Contains multiple approaches to emulating a non-ideal castor wheel. You can
uncomment the commented-out "helper()" methods. The essence was captured there.
Non of these approaches work.

This class sends visualization messages for debugging, so it is more clutterred
than it needs to be. Once everything worked, I was going to tidy this up. It 
never worked.

*/

#include "drive_modes/castor.h"

CastorMode::CastorMode() {
    this->pub_beta = nh.advertise<std_msgs::Float64>("/castor/beta", 100);
	this->pub_castor_init = nh.advertise<robotrainer_controllers::CastorParams>(
		"/castor/init", 100);

    icr_marker.header.frame_id = "platform";
    icr_marker.header.stamp = ros::Time::now();
    icr_marker.ns = "dm_markers_ns";
    icr_marker.id = 14281;
    icr_marker.type = visualization_msgs::Marker::SPHERE;
    icr_marker.action = visualization_msgs::Marker::ADD;

    icr_marker.pose.orientation.x = 0.0;
    icr_marker.pose.orientation.y = 0.0;
    icr_marker.pose.orientation.z = 0.0;
    icr_marker.pose.orientation.w = 1.0;

    icr_marker.scale.x = 0.1;
    icr_marker.scale.y = 0.1;
    icr_marker.scale.z = 0.1;

    icr_marker.pose.position.x = x;
    icr_marker.pose.position.y = 0.0;
    icr_marker.pose.position.z = 0.0;

    icr_marker.color.r = 0.0f;
    icr_marker.color.g = 0.0f;
    icr_marker.color.b = 0.0f;
    icr_marker.color.a = 1.0;

    icr_marker.lifetime = ros::Duration();

	pub_icr_marker = nh.advertise<visualization_msgs::Marker>("castor_icr_visu", 1);
}

void CastorMode::set_parameters(
robotrainer_controllers::DriveModeParameters& dm_params) {
	dm_params.castor.alpha = M_PI;
	dm_params.castor.d = 0.15;
	dm_params.castor.l = 0.35;
	dm_params.castor.r = 0.05;

    dm_params.virtual_axle.y = 0;
    dm_params.virtual_axle.a = 0;

    this->diff_mode.set_parameters(dm_params);


	a = dm_params.castor.alpha;
    b = dm_params.castor.beta;
	d = dm_params.castor.d;
	l = dm_params.castor.l;
	r = dm_params.castor.r;

    VAx = dm_params.virtual_axle.x;
    VAy = dm_params.virtual_axle.y;
    VAa = dm_params.virtual_axle.a;

    p2 = dm_params.r_min;
//	if (p2 < 0) p2 = 0; 
//	if (p2 > 1) p2 = 1; 

	p = dm_params.p;
	if (p < 0) p = 0; 
	if (p > 1) p = 1; 

	pub_castor_init.publish(dm_params.castor);
} 

void CastorMode::helper(const Eigen::Vector3d& dXI, Eigen::Vector3d& newdXI, Eigen::Vector3d& CdXI) {
    // Calculate dβ/dt for the ideal castor wheel.
    Eigen::Vector3d tmp;
    tmp << std::cos(a+b), std::sin(a+b), (d+l)*std::sin(b);
    double db = - tmp.dot(dXI) / d; // dβ/dt

    // Castor not ideal, i.e. β resists to change.
    db *= p; 

    // So that this->b gets updated.
    CdXI[1] = db;


    // Angle between (0,0)-(1,0) and J-C where J is the castor joint, C is the 
    // contact point between wheel and ground.
    const double angle = a + b - M_PI/2;

    // Unit vector in that direction.
    Eigen::Vector3d vc;
    vc << std::cos(angle), std::sin(angle), 0;

    // A unit vector perpendicular to dξ/dt's linear part.
    Eigen::Vector3d vp;
    vp[0] = dXI[1];
    vp[1] = -dXI[0];
    vp[2] = 0;
    if (vp.norm() < 0.0001) return; //avoid div by zero
    vp /= vp.norm();

    // Make sure vp and vc are more than 90 degrees apart.
    if (vp.dot(vc) < 0) vp *= -1;
    // Note: Ensuring "less than 90 degrees" is also a valid approach. But that
    // would mean, that φ resists to change more than β. Which might require
    // adjusting the following code.

    // Calculate the magnitude of the "disruption" due to the castor.
    // If castor is aligned with the drive command, dξ/dt, there is no 
    // disruption. If they are perpendicular, the disruption is at its maximum.
    // Therefore, dot product makes sense.
    const double magnitude = dXI.dot(vc);

    // Introduce this disruption to dξ/dt. You can adjust the effect through the
    // parameter p2.
    ROS_INFO("p2: %lf mag: %lf vp: %lf %lf %lf", p2, magnitude, vp[0], vp[1], vp[2]);
    newdXI = dXI + p2 * magnitude * vp;
    ROS_INFO("%lf %lf %lf %lf %lf %lf", dXI[0], dXI[1], dXI[2], newdXI[0], newdXI[1], newdXI[2]);

    // We still have a virtual axle we have to consider.
    double vx = newdXI[0];
    double vy = newdXI[1];
    double va = newdXI[2];
    this->diff_mode.apply(vx, vy, va);
    newdXI << vx, vy, va;
}

void CastorMode::dr_apply(double& vx, double& vy, double& va) {
    // Handle is at (-0.348, 0)
    // vy -= 0.348 * va;

    //////// <core>
    Eigen::Vector3d dXI;
    dXI << vx, vy, va;
    Eigen::Vector3d newdXI, CdXI;
    helper(dXI, newdXI, CdXI);

    ROS_INFO("%lf old b: ", b);
    b += CdXI[1];
    ROS_INFO("               %lf new b: ", b);
    vx = newdXI[0];
    vy = newdXI[1];
    va = newdXI[2];
    //////// </core>


    // For visualization
	msg_beta.data = b;
	pub_beta.publish(msg_beta);
//    icr_marker.pose.position.x = icr_x;
//    icr_marker.pose.position.y = icr_y;
//	pub_icr_marker.publish(icr_marker);
}

////////////////////////////////////////////////////////////////////////////////
// YET ANOTHER APPROACH TO CASTOR
////////////////////////////////////////////////////////////////////////////////
//   void CastorMode::helper(const Eigen::Vector3d& dXI, Eigen::Vector3d& newdXI, Eigen::Vector3d& CdXI) {
//       // Aliases
//       const double A = a;
//       const double D = d;
//       const double L = l;
//       const double R = r;
//       double B = b; // β is not constant for castors.
//   
//       // This matrix maps platform movement to castor movement.
//       Eigen::Matrix<double, 3, 3> C;
//       C <<
//        sin(A + B)/R  , -cos(A + B)/R  ,          -(L*cos(B))/R,
//       -cos(A + B)/D  , -sin(A + B)/D  ,    -(sin(B)*(D + L))/D,
//        cos(A + B)/D  ,  sin(A + B)/D  , (sin(B)*(D + L))/D + 1;
//   
//       // colspace(M_dif), i.e. basis vectors for M_dif's admissible plane in 
//       // d[x,y,θ]/dt:
//       Eigen::Vector3d b_M_dif_1;
//       b_M_dif_1 << 1, 0,  sin(VAa)/(VAx*cos(VAa)+VAy*sin(VAa));
//       Eigen::Vector3d b_M_dif_2;
//       b_M_dif_2 << 0, 1, -cos(VAa)/(VAx*cos(VAa)+VAy*sin(VAa));
//   
//       // Basis vectors defining the admissible plane in d[φ, β, γ]/dt.
//       Eigen::Vector3d b_C_1 = C * b_M_dif_1;
//       Eigen::Vector3d b_C_2 = C * b_M_dif_2;
//   
//       // Normalize basis vectors
//       b_C_1 /= b_C_1.norm();
//       b_C_2 /= b_C_2.norm();
//   
//       // What the castor would do if it was ideal.
//       CdXI = C * dXI;
//   
//       // But it is not ideal. Let's mess with dβ/dt.
//       CdXI[1] *= p;
//   
//       // But this throws us off the admissible plane, probably. So, project.
//       // First component of the admissible movement in castor's space.
//       Eigen::Vector3d a_C_1 = ((double)(b_C_1.adjoint() * CdXI)) * b_C_1;
//       Eigen::Vector3d a_C_2 = ((double)(b_C_2.adjoint() * CdXI)) * b_C_2;
//       // A new admissible vector in castor's space.
//       Eigen::Vector3d a_C = a_C_1 + a_C_2;
//   
//       // Move back to platform's space.
//       newdXI = C.inverse() * a_C;
//   }

////////////////////////////////////////////////////////////////////////////////
// YET ANOTHER APPROACH TO CASTOR
////////////////////////////////////////////////////////////////////////////////
// void CastorMode::helper(const Eigen::Vector3d& dXI, Eigen::Vector3d& newdXI, Eigen::Vector3d& CdXI) {
//     const double S = std::sin(a+b);
//     const double C = std::cos(a+b);
// 
//     Eigen::Vector3d tmp;
// 
// 
//     // Would be values for ideal castor.
// 
//     tmp << S, -C, -l*std::cos(b);
//     const double rdf = (double)(tmp.adjoint() * dXI); // r*dφ/dt
// 
//     tmp << C, S, (d+l)*std::sin(b);
//     double db = (double)(tmp.adjoint() * dXI) / -d; // dβ/dt
// 
//     
//     // Castor not ideal 
//     db *= p;
// 
//     // Update castor direction
//     CdXI[1] = b;
//     
//     Eigen::Matrix<double, 3, 3> M;
//     M << S  ,  -C  ,  -l * std::cos(b)       ,
//          C  ,   S  ,  (d + l) * std::sin(b)  ,
//          0  ,   1  ,  VAx                    ; 
// 
//     tmp << rdf, -d * db, -l*std::cos(b);
//     newdXI = M.inverse() * tmp;
// }

/*
void CastorMode::helper(const Eigen::Vector3d& dXI, Eigen::Vector3d& newdXI, Eigen::Vector3d& CdXI) {
    const double S = std::sin(a+b);
    const double C = std::cos(a+b);

    Eigen::Vector3d tmp;

    tmp << C, S, (d+l)*std::sin(b);
    double db = (double)(tmp.adjoint() * dXI) / -d; // dβ/dt

    // Castor not ideal 
    db *= p;

    // Update castor direction
    CdXI[1] = db;


    newdXI = dXI;
    

    // Express dξ/dt at castor.
    newdXI[1] -= newdXI[2] * l;

    // Unit vector in castor direction.
    Eigen::Vector3d u;
    u << std::cos(a+b+M_PI/2), std::sin(a+b+M_PI/2), 0;

    Eigen::Vector3d v;
    v << std::cos(a+b), std::sin(a+b), 0;

    // Where the jammed castor would go. This effect is parametrized thru p2.
    Eigen::Vector3d change = p2 * newdXI.dot(v) * u; 
    if (std::cos(change.dot(newdXI)) < 0) change *= -1;

    // Apply the change.
    newdXI += change;

    // Still need to ensure virtual axle.
    double vx, vy, va;
    vx = newdXI[0];
    vy = newdXI[1];
    va = newdXI[2];
                                //    vy += std::cos(b) * this->norm(vx, vy, va) * p2;
    diff_mode.apply(vx, vy, va);

    newdXI << vx, vy, va;
}
*/
