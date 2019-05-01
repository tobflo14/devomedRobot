#include "pd_quat.hpp"
#include "utils.hpp"

#include <iomanip>

// Creating the controller for first tme
PD_Quat::PD_Quat(double pos_gain, double der_gain)
{
    init();
    this->Kp    = this->Kp * pos_gain;
    this->Kd    = this->Kd * der_gain;
}

// Setting all local variables to zero, i.e. initiating all values
void PD_Quat::init()
{
    this->Kp       = Eigen::Matrix3d::Identity();
    this->Kd       = Eigen::Matrix3d::Identity();

    this->state = idle;

    // All 3d vectors set to zero
    this->err_q         = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    this->ref_q         = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    this->ref_zero_q    = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
}

// Takes in current measured state and sets it to internal variable
void PD_Quat::updateMeas(Eigen::Quaterniond meas)
{
    this->meas = meas;
}

// First run, i.e. when button is pushed.
// Takes in orientation of now, sets internal referance at time zero.
void PD_Quat::updateRefZero(Eigen::Quaterniond ref_zero, void* arg)
{   
    shared_memory *data = (shared_memory *)arg;
    this->ref_zero_q = ref_zero;
    data->robot_qua_ref_zero  = ref_zero;
}

// Updates wanted orientation (ref_q) by taking in a delta QUAT
// and adding it to the reference ZERO value
void PD_Quat::updateRef(Eigen::Quaterniond delta, bool first_run, void* arg)
{   
    // Adding delta to ZERO ref -> new wanted orientation in quat
    // this->ref_q = this->ref_zero_q * delta;

    // q_WNT = conj( conj(q_ref_zero) * delta )
    this->ref_q = conj_of_quaternion(conj_of_quaternion(this->ref_zero_q) * delta);

    // Adding limitation to max orientaion of gripper
	// Simple approach by converting to euler and checking limits
        // Eigen::Vector3d ref_euler = quaternion_to_euler(this->ref_q);
        // bool orient_limit = false;
        // for (int i = 0; i < 3; i++)
        // {
        //     if (ref_euler[i] <  orient_min[i])
        // 	   {
        // 		   ref_euler[i] = orient_min[i];
        // 		   orient_limit = true;
        // 	   }
        // 	   else if (ref_euler[i] >  orient_max[i])
        // 	   {
        // 		   ref_euler[i] = orient_max[i];
        // 		   orient_limit = true;
        // 	   }
        // }
        
        // if (orient_limit == true)
        // {
        // 	   this->ref_q = Eigen::Quaterniond(ref_euler)
        // }

    // Handel first run
    if (first_run == true)
    {
        this->ref_q = delta;
    }

    shared_memory *data = (shared_memory *)arg;
    data->robot_qua_ref  = this->ref_q;
}

// Updates the error between wanted orientation (referance) and measured orientation.
// If first run: update previous error to current error.
void PD_Quat::updateErr(bool first_run, void* arg)
{
    this->err_q = this->ref_q * this->meas.conjugate();

    // Note from Fersk et al. [NTNU] (2013) ECC in CH:
    // Full Quaternion Based Attitude Contorl for a Quadrotor.
    // on eq. (20). Taking into account the use of only:
    // err.x(), err.y(), err.z(), and lose of err.w().
    if(this->err_q.w() < 0)
    {
        this->err_q = this->err_q.conjugate();
    }

    // Handel first run
    if (first_run == true)
    {
        this->err_q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    }  

    shared_memory *data = (shared_memory *)arg;
    data->robot_qua_err = this->err_q;    
}

// Calculation of new oriantation speed w (omega)
    // Use a PD-regulator for new velocity gains
    // Regulator is based eq. (21) on Fersk et al. [NTNU] (2013) ECC in CH:
    // Full Quaternion Based Attitude Contorl for a Quadrotor.
    // A greate article for understadning quaternion PD-controllers and math.
Eigen::Vector3d PD_Quat::calcVel(Eigen::Vector3d w_curr, void* arg)
{
    shared_memory *data = (shared_memory *)arg;
    // Update error
    PD_Quat::updateErr(false, data);

    //printf("err_q [%+.3f %+.3f %+.3f %+.3f] \n",
        //       this->err_q.w(), this->err_q.x(), this->err_q.y(), this->err_q.z());

    // // Calculating Transformation matrix, quaternion to omega
        // Eigen::MatrixXd T_q = angular_vel_transform_quat(this->err_q);
        // Eigen::MatrixXd T_q_t = T_q.transpose();

    Eigen::MatrixXd T_e = angular_vel_transform_euler(w_curr);
    Eigen::MatrixXd T_e_t = T_e.transpose();

    // Checking quaternion conversions back to euler:
        // Eigen::Vector3d print_ref_q = quaternion_to_euler(this->ref_q);
        // Eigen::Vector3d print_ref_q_zero = quaternion_to_euler(this->ref_zero_q);
        // Eigen::Vector3d print_meas_q= quaternion_to_euler(this->meas);
        // Eigen::Vector3d print_err_q = quaternion_to_euler(this->err_q);
        // printf("E_ref_q [%+.3f %+.3f %+.3f] E_zero_q [%+.3f %+.3f %+.3f] E_meas_q [%+.3f %+.3f %+.3f] E_err_q [%+.3f %+.3f %+.3f]\n",
        //             print_ref_q[0], print_ref_q[1], print_ref_q[2],
        //             print_ref_q_zero[0], print_ref_q_zero[1], print_ref_q_zero[2],
        //             print_meas_q[0], print_meas_q[1], print_meas_q[2],
        //             print_err_q[0], print_err_q[1], print_err_q[2]);

    // Calculating P and D (from eq. (21), meantioned in intro)
	Eigen::Vector3d P = Kp * Eigen::Vector3d(this->err_q.x(), this->err_q.y(), this->err_q.z());
	// Eigen::Vector3d P = Kp * T_q_t * Eigen::Vector4d(this->err_q.w(), this->err_q.x(), this->err_q.y(), this->err_q.z());
	Eigen::Vector3d D = Kd * T_e_t * w_curr;
	// Eigen::Vector3d D = Eigen::Vector3d(0.0, 0.0, 0.0);
 
    // NOTE!!! (NEEDS IMPLEMENTAION) from AXL 2018.08.19 17:20
        // We need to add something to handle crossing over from neg(-) pi() till pos(+) pi().
        // This is due to quaternion design and us not understanding quite how to add a difference
        // I suspect. More informtion on topic is in Kreyszig (9th ed.) p. 377 Vector Product def.
        //
        // Mostlikly this is an effect of correponding/contracted axis change.

    // New omega
	Eigen::Vector3d w_nxt = P + D;
    // printf("w_nxt [%+.4f %+.4f %+.4f] Reg: P[%+.4f %+.4f %+.4f] D[%+.4f %+.4f %+.4f]\n",
    //             w_nxt[0], w_nxt[1], w_nxt[2], P[0], P[1], P[2], D[0], D[1], D[2]);

	return w_nxt;
}

// void PD_Vec3d::stateMachine(bool command)
void PD_Quat::stateMachine(bool command)
{
    // Update references depending on robot command state
    if (this->state == idle && command == true)
    {
        std::cout << "   ROT: Robot is initiating rotation" << std::endl;
        this->state = initiate_move;
    }
    else if (this->state == initiate_move && command == true)
    {
        printf("%s   ROT: Robot is rotating%s\n", KBLU, KNRM);
        this->state  = move;
    }
    else if (this->state == move && command == false)
    {
        std::cout << "ROT: Robot is idle" << std::endl;
        this->state =idle;
    }
    else if (this->state == initiate_move && command == false)
    {
        printf("%sROT: Warring! Initiate and no move command%s\n", KRED, KNRM);
        this->state =idle;
    }
}
