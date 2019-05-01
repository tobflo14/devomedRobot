#include "pd_vec3d.hpp"

// Color to printf("%s YOUR TEXT %s\n", KRED, KNRM);
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

// Creating the controller for first time
PD_Vec3d::PD_Vec3d(double mass_factor, double pos_gain, double der_gain)
{
    init();
    K      = mass_factor;
    Kp     = pos_gain;
    Kd     = der_gain;
}

// Setting all local variables to zero, i.e. initiating all values
void PD_Vec3d::init()
{
    this->K      = 0.0;
    this->Kp     = 0.0;
    this->Kd     = 0.0;

    this->state = idle;

    // All 3d vectors set to zero
    this->vel           = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->vel_prev      = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->err           = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->err_prev      = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->err_dot       = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->force         = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->meas          = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->ref           = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->ref_zero      = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->acc           = Eigen::Vector3d(0.0, 0.0, 0.0);

}

// Takes in current measured state and sets it to internal variable
void PD_Vec3d::updateMeas(Eigen::Vector3d meas)
{
    this->meas = meas;
}

// Updates wanted postion (ref) by taking in a delta XYZ
// and adding it to the reference ZERO value
void PD_Vec3d::updateRef(Eigen::Vector3d delta, bool first_run, void* arg)
{   
    // Adding delta to ZERO ref -> new wanted position in XYZ
    this->ref = this->ref_zero + delta;

    // Setting boundries for movement posibilities
    // Caping referance so it cannot be set to more than a given value
    static const Eigen::Vector3d pos_min(-0.2, 0.2, 0.1);
    static const Eigen::Vector3d pos_max(0.6, 0.5, 0.7);
    static const double r_max = 0.75;
    // Shperical constraint
    double r_nxt = sqrt(pow(this->ref[0], 2) + pow(this->ref[1], 2) + pow(this->ref[2], 2));   
    if(r_nxt > r_max)
    {
        this->ref = this->ref * (r_max/r_nxt);
    }    
    // Cartisian constraint
    for(int i = 0; i < 3; i++)
    {
        if(this->ref[i] <  pos_min[i])
        {
            this->ref[i] = pos_min[i];
        }
    }

    // Handel first run
    if (first_run == true){    this->ref = delta;    }

    shared_memory *data = (shared_memory *)arg;
    data->robot_pos_ref  = this->ref;
}

// First run, i.e. when button is pushed.
// Takes in position of now, sets internal referance at time zero.
void PD_Vec3d::updateRefZero(Eigen::Vector3d ref_zero, void* arg)
{   
    shared_memory *data = (shared_memory *)arg;
    this->ref_zero = ref_zero;
    data->robot_pos_ref_zero = ref_zero;
}


// Updates the error between wanted point(referance) and measured point.
// If first run: update previous error to current error.
void PD_Vec3d::updateErr(bool first_run, void* arg)
{
    this->err = this->ref - this->meas;
    if (first_run == true){   this->err_prev = this->err;   }

    shared_memory *data = (shared_memory *)arg;
    data->robot_pos_err = this->err;
}

std::vector<double> PD_Vec3d::returnData() {
    return {
        this->Kp * this->err[0],
        this->Kd * this->err_dot[0],
        this->force[0]
        };
}

// ??????????????? HOW DOES THIS WORK ????????????????????
// Calculation of new cartesian speed (speed in XXZ)
// Use a PD-regulator for new velocity gains
Eigen::Vector3d PD_Vec3d::calcVel(double dT, void* arg)
{
    shared_memory *data = (shared_memory *)arg;
    // Update error
    PD_Vec3d::updateErr(false, data);

    // Deriviative to calculate current speed
    this->err_dot = (this->err - this->err_prev) / dT;

    // PD reg to find accelleration input
    this->force = this->K*(this->Kp * this->err + this->Kd * this->err_dot);

    // this->acc = this->force * this->K;

    // // Limit requested acceleration to a defined maximum. Makes QP solver more reliable.
    // double abs_acc = sqrt(pow(this->acc[0],2) + pow(this->acc[1],2) + pow(this->acc[2],2));
    // if ( abs_acc > MAX_ACCEL )
    // {
    //     double factor = MAX_ACCEL/abs_acc;
    //     this->acc *= factor;
    // }

    // // Acceleration do speed by discretization
    // this->vel = this->vel_prev + this->acc * dT;
    // // Limit requested acceleration to a defined maximum. Makes QP solver more reliable.
    // double abs_vel = sqrt(pow(this->vel[0],2) + pow(this->vel[1],2) + pow(this->vel[2],2));
    // if ( abs_vel > MAX_VEL )
    // {
    //     double factor = MAX_VEL/abs_vel;
    //     this->vel *= factor;
    // }

    // this->vel_prev = this->vel;
    this->err_prev = this->err;

    return this->force;
}

// void PD_Vec3d::stateMachine(bool command)
void PD_Vec3d::stateMachine(bool command)
{
    // Update references depending on robot command state
    if (this->state == idle && command == true)
    {
        this->state = initiate_move;
        std::cout << "   XYZ: Robot is initiating move" << std::endl;
    }
    else if (this->state == initiate_move && command == true)
    {
        this->state  = move;
        printf("%s   XYZ: Robot is moving%s\n", KBLU, KNRM);
        // std::cout << "XYZ: Robot is moving" << std::endl;
    }
    else if (this->state == move && command == false)
    {
        this->state =idle;
        std::cout << "XYZ: Robot position is idle" << std::endl;
    }
    else if (this->state == initiate_move && command == false)
    {
        printf("%sXYZ: Warring! Initiate and no move command%s\n", KRED, KNRM);
        // std::cout << "XYZ: Warring!! Initiate and no move command" << std::endl;
        this->state =idle;
    }
}
