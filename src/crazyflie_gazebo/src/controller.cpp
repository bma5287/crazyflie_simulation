#include "controller.h"
#include <iomanip>      // provides std::setprecision
#include <math.h>
#include <algorithm>
#include <stdint.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


void Controller::Load(int port_number_gazebo)
{   cout << setprecision(4);
    cout << fixed;
    isRunning = true;

    // =========== Gazebo Connection =========== //
    socket_Gazebo = socket(AF_INET, SOCK_DGRAM, 0);
    fd_gazebo_SNDBUF = 16;         // Motorspeeds to Gazebo 4 doubles (16 Bytes)
    fd_gazebo_RCVBUF = 112;        // State info from Gazebo 14 doubles (112 Bytes)

    if (setsockopt(socket_Gazebo, SOL_SOCKET, SO_SNDBUF, &fd_gazebo_SNDBUF, sizeof(fd_gazebo_SNDBUF))<0)
        cout<<"socket_Gazebo setting SNDBUF failed"<<endl;

    if (setsockopt(socket_Gazebo, SOL_SOCKET, SO_RCVBUF, &fd_gazebo_RCVBUF, sizeof(fd_gazebo_RCVBUF))<0)
        cout<<"socket_Gazebo setting RCVBUF failed"<<endl;

    port_number_gazebo_ = port_number_gazebo;

    memset(&sockaddr_local_Gazebo, 0, sizeof(sockaddr_local_Gazebo));
    sockaddr_local_Gazebo.sin_family = AF_INET;
    sockaddr_local_Gazebo.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("127.0.0.1");
    sockaddr_local_Gazebo.sin_port = htons(18070);

    // Check for successful binding of socket to port
    if ( bind(socket_Gazebo, (struct sockaddr*)&sockaddr_local_Gazebo, sizeof(sockaddr_local_Gazebo)) < 0)
        cout<<"Socket binding to Gazebo failed"<<endl;
    else
        cout<<"Socket binding to Gazebo succeeded"<<endl; 

    memset(&sockaddr_remote_Gazebo, 0, sizeof(sockaddr_remote_Gazebo));
    sockaddr_remote_Gazebo.sin_family = AF_INET;
    sockaddr_remote_Gazebo.sin_addr.s_addr = htonl(INADDR_ANY);
    sockaddr_remote_Gazebo.sin_port = htons(port_number_gazebo_);
    sockaddr_remote_gazebo_len = sizeof(sockaddr_remote_Gazebo);
    
    // Motorspeed send test
    float msg[4] = {100.0,100.0,100.0,100.0};
    int len = 0;
    for(int k=0; k<2; k++)
        // To Gazebo socket, send msg of len(msg)
        len = sendto(socket_Gazebo, msg, sizeof(msg),0, (struct sockaddr*)&sockaddr_remote_Gazebo, sizeof(sockaddr_remote_Gazebo));
    if(len>0)
        cout<<"Send initial motor speed ["<<len<<" bytes] to Gazebo Succeeded! \nAvoiding threads mutual locking"<<endl;
    else
        cout<<"Send initial motor speed to Gazebo FAILED! Threads will mutual lock"<<endl;




    // =========== Python RL Connection =========== //
    fd_RL = socket(AF_INET, SOCK_DGRAM, 0);
    fd_RL_SNDBUF = 112;        // 112 bytes is 14 double
    fd_RL_RCVBUF = 40;         // 40 bytes is 5 double

    // Set expected buffer for incoming/outgoing messages to RL
    if (setsockopt(fd_RL, SOL_SOCKET, SO_SNDBUF, &fd_RL_SNDBUF, sizeof(fd_RL_SNDBUF))<0)
        cout<<"fd_RL setting SNDBUF failed"<<endl;
    if (setsockopt(fd_RL, SOL_SOCKET, SO_RCVBUF, &fd_RL_RCVBUF, sizeof(fd_RL_RCVBUF))<0)
        cout<<"fd_RL setting RCVBUF failed"<<endl;

    memset(&sockaddr_local_RL, 0, sizeof(sockaddr_local_RL)); // Not sure what this does
    sockaddr_local_RL.sin_family = AF_INET; // IPv4 Format
    sockaddr_local_RL.sin_port = htons(18060); // RL Port number
    sockaddr_local_RL.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("127.0.0.1");
    

    if (bind(fd_RL, (struct sockaddr*)&sockaddr_local_RL, sizeof(sockaddr_local_RL))<0)
        cout<<"Socket binding to crazyflie_env.py failed"<<endl;
    else
        cout<<"Socket binding to crazyflie_env.py succeeded"<<endl;
    
    
    receiverThread_gazebo = std::thread(&Controller::recvThread_gazebo, this);
    //senderThread_gazebo = std::thread(&Controller::sendThread_gazebo, this);
    receiverThread_RL = std::thread(&Controller::recvThread_RL, this);
    controllerThread = std::thread(&Controller::controlThread, this);

    queue_states = moodycamel::BlockingReaderWriterQueue<StateFull>(5);
    queue_motorspeed = moodycamel::BlockingReaderWriterQueue<MotorCommand>(5);
}




void Controller::recvThread_gazebo()
{
    double state_full[14];
    StateFull state_full_structure;

    while(isRunning)
    {
        //cout<<"[recvThread_gazebo] Receiving crazyflie states from Gazebo"<<endl;
        int len = recvfrom(socket_Gazebo, state_full, sizeof(state_full),0, (struct sockaddr*)&sockaddr_remote_Gazebo, &sockaddr_remote_gazebo_len);

        /*if(len>0)
        {
            cout<<"Enqueue full State:";
            for(int k=0;k<13;k++)
                cout<<state_full[k]<<", ";
            cout<<"\n";
        }*/

        memcpy(state_full_structure.data, state_full, sizeof(state_full));
        queue_states.enqueue(state_full_structure);
        sendto(fd_RL, state_full, sizeof(state_full),0, (struct sockaddr*)&sockaddr_remote_rl, sockaddr_remote_rl_len);
    }
}


void Controller::sendThread_gazebo()
{
    float motorspeed[4];
    MotorCommand motorspeed_structure;

    while(isRunning)
    {
        queue_motorspeed.wait_dequeue(motorspeed_structure);
        memcpy(motorspeed, motorspeed_structure.data, sizeof(motorspeed));

        //cout<<"[recvThread_RL] sending motor speed to Gazebo"<<endl;
        int len=sendto(socket_Gazebo, motorspeed, sizeof(motorspeed),0, (struct sockaddr*)&sockaddr_remote_Gazebo, sockaddr_remote_gazebo_len);
        if(len>0)
            cout<<"[recvThread_RL] sent motor speed ["<<motorspeed[0]<<", "<<motorspeed[1]<<", "<<motorspeed[2]<<", "<<motorspeed[3]<<"]"<<endl;
        else
            cout<<"[recvThread_RL] sending motor speed to Gazebo FAILED!"<<endl;
    }
    
}




void Controller::recvThread_RL()
{
    float motorspeed_fake[4] = {0,0,0,0};

    while(isRunning)
    {
        //cout<<"[recvThread_RL] Receiving command from RL"<<endl;
        int len = recvfrom(fd_RL, control_cmd_recvd, sizeof(control_cmd_recvd),0, (struct sockaddr*)&sockaddr_remote_rl, &sockaddr_remote_rl_len);


        if(control_cmd_recvd[0]>10) // If header is 11 then enable sticky
        {
            motorspeed_fake[0] = -control_cmd_recvd[0];
            motorspeed_fake[1] = control_cmd_recvd[1];
            //cout<<"Send sticky command command: "<< motorspeed_fake[0]<<", "<<motorspeed_fake[1]<<endl;
            sendto(socket_Gazebo, motorspeed_fake, sizeof(motorspeed_fake),0, (struct sockaddr*)&sockaddr_remote_Gazebo, sockaddr_remote_gazebo_len);

            /*if (control_cmd_recvd[1]<0.5)     // reset_world signal
            {
                control_cmd_recvd[0] = 2; control_cmd_recvd[1] = 0; control_cmd_recvd[2] = 0; control_cmd_recvd[3] = 0; control_cmd_recvd[4] = 0;
            }*/
            
        }
    }
}





Vector3d dehat(Matrix3d a_hat) // Input a skew-symmetric matrix and output corresponding vector
{

    /* Convert skew-symmetric matrix a_hat into vector a

    a_hat = [  0   -a3   a2 ]
            [  a3   0   -a1 ]
            [ -a2   a1   0  ]


    a = [ a1 ] 
        [ a2 ] 
        [ a3 ]

    */

    Vector3d a;
    Matrix3d tmp;

    tmp = (a_hat - a_hat.transpose())/2; // Not sure why this is done

    a(0) = tmp(2,1);
    a(1) = tmp(0,2);
    a(2) = tmp(1,0);

    return a;
}

Matrix3d hat(Vector3d a) // Input a hat vector and output corresponding skew-symmetric matrix
{ 
  // You hat a vector and get a skew-symmetric matrix
  // You dehat/dehat a skew-symmetric matrix and get a vector

    /* Convert a into skew symmetric matrix a_hat
    a = [ a1 ] 
        [ a2 ] 
        [ a3 ]
 
    a_hat = [  0   -a3   a2 ]
            [  a3   0   -a1 ]
            [ -a2   a1   0  ]
    ]
    */
    Matrix3d a_hat;
    a_hat(2,1) =  a(0);
    a_hat(1,2) = -a(0);

    a_hat(0,2) =  a(1);
    a_hat(2,0) = -a(1);

    a_hat(1,0) =  a(2);
    a_hat(0,1) = -a(2);

    return a_hat;
}














void Controller::controlThread()
{

    // =========== Controller Explanation =========== //
    // https://www.youtube.com/watch?v=A27knigjGS4&list=PL_onPhFCkVQhuPiUxUW2lFHB39QsavEEA&index=46
    // Derived from DOI's: 10.1002/asjc.567 (T. Lee) & 10.13140/RG.2.2.12876.92803/1 (M. Fernando)
    typedef Matrix<double, 3, 3, RowMajor> RowMatrix3d; 

    
    StateFull state_full_structure;
    MotorCommand motorspeed_structure;

    float motorspeed[4];
    double state_full[14];
    

    int type; // Command type {1:Pos, 2:Vel, 3:Att, 4:Omega, 5:Stop}
    double ctrl_flag; // On/Off switch for controller
    double control_cmd[5];
    Vector3d control_vals;
    

    // State Declarations

    Vector3d pos; // Current position [m]
    Vector3d vel; // Current velocity [m]
    Vector4d quat_Eig; // Current attitude [rad] (quat form)
    Vector3d eul; // Current attitude [rad] (roll, pitch, yaw angles)
    Vector3d omega; // Current angular velocity [rad/s]

    

    // Default desired States
    Vector3d x_d_Def(0,0,0.23); // Pos-desired (Default) [m] 
    Vector3d v_d_Def(0,0,0); // Velocity-desired (Default) [m/s]
    Vector3d a_d_Def(0,0,0); // Acceleration-desired (Default) [m/s]
    Vector3d b1_d_Def(1,0,0); // Desired global pointing direction (Default)
    Vector3d omega_d_Def(0,0,0);
    

    // State Error and Prescriptions
    Vector3d x_d; // Pos-desired [m] 
    Vector3d v_d; // Velocity-desired [m/s]
    Vector3d a_d; // Acceleration-desired [m/s]

    Matrix3d R_d; // Rotation-desired 
    Matrix3d R_d_custom; // Rotation-desired (ZXY Euler angles)
    Vector3d eul_d; // Desired attitude (ZXY Euler angles) [rad] (roll, pitch, yaw angles)
    Vector3d omega_d; // Omega-desired [rad/s]
    Vector3d domega_d(0,0,0); // Omega-Accl. [rad/s^2]

    Vector3d b1_d; // Desired yaw direction of body-fixed axis (COM to prop #1)
    Vector3d b2_d; // Desired body-fixed axis normal to b1 and b3
    Vector3d b3_d; // Desired body-fixed vertical axis

    Vector3d e_x; // Pos-Error [m]
    Vector3d e_v; // Vel-error  [m/s]
    Vector3d e_R; // Rotation-error [rad]
    Vector3d e_omega; // Omega-error [rad/s]

    


    // Matrix3d J_temp;
    // J_temp << 16.5717,0.8308,0.7183,
    //           0.8308,16.6556,1.8002,
    //           0.7183,1.8002,29.2617; // Sourced from J. Forster
    // J = J_temp*1e-6;




    Vector3d e3(0,0,1); // Global z-axis

    




    Vector3d F_thrust_ideal; // Ideal thrust vector to minimize error   
    Vector3d Gyro_dyn; // Gyroscopic dynamics of system [Nm]
    Vector3d M; // Moment control vector [Nm]
    
    
    Vector4d FM; // Thrust-Moment control vector (4x1)
    Vector4d f; // Propeller thrusts [N]
    double F_thrust;

    

    Vector4d motorspeed_Vec_d; // Desired motorspeeds [rad/s]
    Vector4d motorspeed_Vec; // Motorspeeds [rad/s]


    Vector3d b3; // Body-fixed vertical axis
    Quaterniond q;
    Matrix3d R; // Body-Global Rotation Matrix
    


    
    // Controller Values
    Vector4d Ctrl_Gains; 
    double kp_x = 0.1;   // Pos. Gain
    double kd_x = 0.08;  // Pos. derivative Gain
    double kp_R = 0.05;  // Rot. Gain // Keep checking rotational speed
    double kd_R = 0.005; // Rot. derivative Gain

    double kp_omega = 0.0005; 
    // Omega proportional gain (similar to kd_R but that's for damping and this is to achieve omega_d)
    // (0.0003 Fully saturates motors to get to omega_max (40 rad/s))
    // kd_R is great for stabilization but for flip manuevers it's too sensitive and 
    // saturates the motors causing instability during the rotation

    // Controller Flags
    double kp_xf = 1; // Pos. Gain Flag
    double kd_xf = 1; // Pos. derivative Gain Flag
    double kp_Rf = 1; // Rot. Gain Flag
    double kd_Rf = 1; // Rot. derivative Gain Flag

    


    double yaw; // Z-axis [rad/s]
    double roll; // X-axis [rad/s]
    double pitch; // Y-axis [rad/s]



    // might need to adjust weight to real case (sdf file too)
    double m = 0.026 + 0.00075*4; // Mass [kg]
    double g = 9.8066; // Gravitational acceleration [m/s^2]
    double t = 0; // Time from Gazebo [s]
    unsigned int t_step = 0; // t_step counter



    // System Constants
    double d = 0.040; // Distance from COM to prop [m]
    double d_p = d*sin(M_PI/4);

    double kf = 2.21e-8; // Thrust constant [N/(rad/s)^2]
    double c_Tf = 0.00612; // Moment Constant [Nm/N]

    Matrix3d J; // Rotational Inertia of CF
    J<< 1.65717e-05, 0, 0,
        0, 1.66556e-05, 0,
        0, 0, 2.92617e-05;

    Matrix4d Gamma; // Thrust-Moment control vector conversion matrix
    Gamma << 1,     1,     1,     1, // Motor thrusts = Gamma*Force/Moment vec
             d_p,   d_p,  -d_p,  -d_p, 
            -d_p,   d_p,   d_p,  -d_p, 
            c_Tf,  -c_Tf, c_Tf,  -c_Tf;
    Matrix4d Gamma_I = Gamma.inverse(); // Calc here once to reduce calc load



  
    
    // 1 is on | 0 is off by default
    // double att_control_flag = 0; // Controls implementation
    double flip_flag = 1; // Controls thrust implementation
    double motorstop_flag = 0; // Controls stop implementation
    // double flip_flag = 1; // Controls gyroscopic dynamics in moment equation



   
    double w; // Angular frequency for trajectories [rad/s]
    double b; // Amplitude for trajectories [m]

    // =========== Trajectory Definitions =========== //
    x_d << x_d_Def;
    v_d << v_d_Def;
    a_d << a_d_Def;
    b1_d << b1_d_Def;

    while(isRunning)
    {

        
      
        // if (t>=8.1){ // Vertical Petal Traj.
        // w = 3.0; // rad/s
        // x_d << (cos(M_PI/2 + t*w)*cos(t))/2, 0, (cos(M_PI/2 + t*w)*sin(t))/2 + 1;
        // v_d << (sin(t*w)*sin(t))/2 - (w*cos(t*w)*cos(t))/2, 0, - (sin(t*w)*cos(t))/2 - (w*cos(t*w)*sin(t))/2;
        // a_d  << (sin(t*w)*cos(t))/2 + w*cos(t*w)*sin(t) + (pow(w,2)*sin(t*w)*cos(t))/2, 0, (sin(t*w)*sin(t))/2 - w*cos(t*w)*cos(t) + (pow(w,2)*sin(t*w)*sin(t))/2;
        // b1_d << 1,0,0;
        // }

        // if (t>=20){ // Horizontal Circle Traj.
        // w = 2.0; // rad/s
        // b = 0.5;
        // x_d << b*cos(t*w), b*sin(t*w), 1.5;
        // v_d << -b*w*sin(t*w), b*w*cos(t*w), 0;
        // a_d << -b*pow(w,2)*cos(t*w), -b*pow(w,2)*sin(t*w), 0;
        // b1_d << 1,0,0;
        // }



        // =========== Control Definitions =========== //
        // Define control_cmd from recieved control_cmd
        if (control_cmd_recvd[0]<10) // Change to != 10 to check if not a sticky foot command
            memcpy(control_cmd, control_cmd_recvd, sizeof(control_cmd)); // Compiler doesn't work without this line for some reason? 
            Map<Matrix<double,5,1>> control_cmd_Eig(control_cmd_recvd); 

        type = control_cmd_Eig(0); // Command type
        control_vals = control_cmd_Eig.segment(1,3); // Command values
        ctrl_flag = control_cmd_Eig(4); // Controller On/Off switch

        switch(type){ // Define Desired Values

            case 0: // Reset all changes and return to home
                x_d << x_d_Def;
                v_d << v_d_Def;
                a_d << a_d_Def;
                b1_d << b1_d_Def;

                omega_d << omega_d_Def;
                kd_R = 0.005; 

                kp_xf=1,kd_xf=1,kp_Rf=1,kd_Rf=1; // Reset control flags
                motorstop_flag=0;
                flip_flag=1;
                // att_control_flag=0;
                break;

            case 1: // Position
                x_d << control_vals;
                kp_xf = ctrl_flag;
                break;

            case 2: // Velocity
                v_d << control_vals;
                kd_xf = ctrl_flag;
                break;

            case 3: // Attitude [Implementation needs to be finished]
                eul_d << control_vals;
                kp_Rf = ctrl_flag;

                // att_control_flag = ctrl_flag;
                break;

            case 4: // Exectute Flip
                kd_R = kp_omega; // Change to flip gain

                omega_d << control_vals;
                kd_Rf = ctrl_flag; // Turn control on
                kp_xf = 0; // Turn off other controllers
                kd_xf = 0;
                kp_Rf = 0;
                
                flip_flag = 0; 
                // Turn gyroscopic dynamics off [maybe not needed?]
                // Turn thrust control off
                // Change e_omega calc

                break;

            case 5: // Stop Motors
                motorstop_flag = ctrl_flag;
                break;

            case 6: // Reassign new control gains
                Ctrl_Gains << control_vals,ctrl_flag;
                kp_x = Ctrl_Gains(0);
                kd_x = Ctrl_Gains(1);
                kp_R = Ctrl_Gains(2);
                kd_R = Ctrl_Gains(3);
                break;
        }

        // =========== State Definitions =========== //

        // Define state_full from recieved Gazebo states and break into corresponding vectors
        queue_states.wait_dequeue(state_full_structure);
        // memcpy(state_full, state_full_structure.data, sizeof(state_full));

        Map<Matrix<double,1,14>> state_full_Eig(state_full_structure.data); // Convert threaded array to Eigen vector     
        pos = state_full_Eig.segment(0,3); // .segment(index,num of positions)
        quat_Eig = state_full_Eig.segment(3,4);
        vel = state_full_Eig.segment(7,3);
        omega = state_full_Eig.segment(10,3);
        t = state_full_Eig(13); 
        




        // =========== Rotation Matrix =========== //
        // R changes Body axes to be in terms of Global axes
        // https://www.andre-gaschler.com/rotationconverter/
        q.w() = quat_Eig(0);
        q.vec() = quat_Eig.segment(1,3);
        R = q.normalized().toRotationMatrix(); // Quaternion to Rotation Matrix Conversion
        
        yaw = atan2(R(1,0), R(0,0)); 
        roll = atan2(R(2,1), R(2,2)); 
        pitch = atan2(-R(2,0), sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
    
        b3 = R*e3; // body vertical axis in terms of global axes


        // =========== Translational Errors & Desired Body-Fixed Axes =========== //
        e_x = pos - x_d; 
        e_v = vel - v_d;

        F_thrust_ideal = -kp_x*kp_xf*e_x + -kd_x*kd_xf*e_v + m*g*e3 + m*a_d; // ideal control thrust vector
        b3_d = F_thrust_ideal.normalized(); // desired body-fixed vertical axis
        b2_d = b3_d.cross(b1_d).normalized(); // body-fixed horizontal axis



        // =========== Rotational Errors =========== // 
        R_d << b2_d.cross(b3_d).normalized(),b2_d,b3_d; // Desired rotational axis
                                                        // b2_d x b3_d != b1_d (look at derivation)

        // if (att_control_flag == 1){ // [Attitude control will be implemented here]
        //     R_d = R_d_custom;
        // }

        e_R = 0.5*dehat(R_d.transpose()*R - R.transpose()*R_d); // Rotational error
        e_omega = omega - R.transpose()*R_d*omega_d; // Ang vel error (Global Reference Frame->Body?)
        // (This is omega - deltaR*omega_d | Not entirely sure how that correlates though)

        if(flip_flag == 0){ // I've just sold my soul by doing this
            e_omega(2) = 0; // Remove yaw control when executing flip
        }



        // =========== Control Equations =========== // 
        F_thrust = F_thrust_ideal.dot(b3)*(flip_flag); // Thrust control value
        Gyro_dyn = omega.cross(J*omega) - J*(hat(omega)*R.transpose()*R_d*omega_d - R.transpose()*R_d*domega_d); // Gyroscopic dynamics
        M = -kp_R*e_R*(kp_Rf) + -kd_R*e_omega*(kd_Rf) + Gyro_dyn; // Moment control vector
        FM << F_thrust,M; // Thrust-Moment control vector



        // =========== Propellar Thrusts/Speeds =========== //
        f = Gamma_I*FM; // Propeller thrusts
        motorspeed_Vec_d = (1/kf*f).array().sqrt(); // Calculated motorspeeds
        motorspeed_Vec = motorspeed_Vec_d; // Actual motorspeeds to be capped


        // Cap motor thrusts between 0 and 2500 rad/s
        for(int k_motor=0;k_motor<4;k_motor++) 
        {
            if(motorspeed_Vec(k_motor)<0){
                motorspeed_Vec(k_motor) = 0;
            }
            else if(isnan(motorspeed_Vec(k_motor))){
                motorspeed_Vec(k_motor) = 0;
            }
            else if(motorspeed_Vec(k_motor)>= 2500){ // Max rotation speed (rad/s)
                // cout << "Motorspeed capped - Motor: " << k_motor << endl;
                motorspeed_Vec(k_motor) = 2500;
            }
        }

        if(b3(2) <= sqrt(2)/2){ // If e3 component of b3 is neg, turn motors off [arbitrary amount]
            motorspeed_Vec << 0,0,0,0;
        }


        if(motorstop_flag == 1){ // Shutoff all motors
            motorspeed_Vec << 0,0,0,0;
        }
        


        if (t_step%50 == 0){ // General Debugging output
        cout << "t: " << t << "\tCmd: " << control_cmd_Eig.transpose() << endl <<
        "kp_x: " << kp_x << " \tkd_x: " << kd_x << " \tkp_R: " << kp_R << " \tkd_R: " << kd_R << "\tkd_R_fl: " << kp_omega << endl <<
        "kp_xf: " << kp_xf << " \tkd_xf: " << kd_xf << " \tkp_Rf: " << kp_Rf << " \tkd_Rf: " << kd_Rf << " \tflip_flag: " << flip_flag << endl <<
        endl <<
        "x_d: " << x_d.transpose() << endl <<
        "v_d: " << v_d.transpose() << endl <<
        "omega_d: " << omega_d.transpose() << endl <<
        endl << 
        "pos: " << pos.transpose() << "\tex: " << e_x.transpose() << endl <<
        "vel: " << vel.transpose() << "\tev: " << e_v.transpose() << endl <<
        "omega: " << omega.transpose() << "\te_w: " << e_omega.transpose() << endl <<
        endl << 
        "R:\n" << R << "\n\n" << 
        "R_d:\n" << R_d << "\n\n" << 
        "Yaw: " << yaw*180/M_PI << "\tRoll: " << roll*180/M_PI << "\tPitch: " << pitch*180/M_PI << endl <<
        "e_R: " << e_R.transpose() << "\te_R (deg): " << e_R.transpose()*180/M_PI << endl <<
        endl <<
        "FM: " << FM.transpose() << endl <<
        "f: " << f.transpose() << endl <<
        endl <<
        "MS_d: " << motorspeed_Vec_d.transpose() << endl <<
        "MS: " << motorspeed_Vec.transpose() << endl <<
        "=============== " << endl; 
        printf("\033c"); // clears console window
        }

     
        Map<RowVector4f>(&motorspeed[0],1,4) = motorspeed_Vec.cast <float> (); // Converts motorspeeds to C++ array for data transmission
        sendto(socket_Gazebo, motorspeed, sizeof(motorspeed),0, // Send motorspeeds to Gazebo -> gazebo_motor_model?
                (struct sockaddr*)&sockaddr_remote_Gazebo, sockaddr_remote_gazebo_len); 

        t_step++;
    }
}



int main()
{
    Controller controller;
    controller.Load(18080); // Run controller as a thread

    while(1)
    {
        sleep(1e7);         // 1e7 is about 100 days
    }
    
    return 0;
}