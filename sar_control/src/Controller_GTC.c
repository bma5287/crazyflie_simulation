#include "Controller_GTC.h"

#define max(a,b) ((a) > (b) ? (a) : (b))


void appMain() {

    while (1)
    {
        #ifdef CONFIG_SAR_SIM

            if (CTRL_Cmd.cmd_rx == true)
            {
                CTRL_Command(&CTRL_Cmd);
                CTRL_Cmd.cmd_rx = false;
            }


        #elif CONFIG_SAR_EXP

            if (appchannelReceiveDataPacket(&CTRL_Cmd,sizeof(CTRL_Cmd),APPCHANNEL_WAIT_FOREVER))
            {
                if (CTRL_Cmd.cmd_rx == true)
                {
                    CTRL_Command(&CTRL_Cmd);
                    CTRL_Cmd.cmd_rx = false;
                }
            }

        #endif
    }
    
}

    
bool controllerOutOfTreeTest() {

  return true;
}


void controllerOutOfTreeInit() {

    #ifdef CONFIG_SAR_EXP
    
    #endif

    controllerOutOfTreeReset();
    controllerOutOfTreeTest();

    // INIT DEEP RL NN POLICY
    X_input = nml_mat_new(4,1);
    Y_output = nml_mat_new(4,1);

    // INIT DEEP RL NN POLICY
    NN_init(&NN_DeepRL,NN_Params_DeepRL);

    consolePrintf("GTC Controller Initiated\n");
}


void controllerOutOfTreeReset() {

    consolePrintf("GTC Controller Reset\n");
    consolePrintf("SAR_Type: %d\n",SAR_Type);
    consolePrintf("Policy_Type: %d\n",Policy);
    J = mdiag(Ixx,Iyy,Izz);

    // RESET INTEGRATION ERRORS
    e_PI = vzero();
    e_RI = vzero();

    // TURN POS/VEL CONTROLLER FLAGS ON
    kp_xf = 1.0f;
    kd_xf = 1.0f;

    // RESET SETPOINTS TO HOME POSITION
    x_d = mkvec(-60.0f, 0.0f, 1.0f);
    v_d = mkvec(0.0f,0.0f,0.0f);
    a_d = mkvec(0.0f,0.0f,0.0f);
    b1_d = mkvec(1.0f,0.0f,0.0f);

    // RESET SYSTEM FLAGS
    Tumbled_Flag = false;
    CustomThrust_Flag = false;
    CustomMotorCMD_Flag = false;
    AngAccel_Flag = false;

    // RESET TRAJECTORY FLAGS
    Traj_Type = NONE;
    resetTraj_Vals(0);
    resetTraj_Vals(1);
    resetTraj_Vals(2);

    // RESET POLICY FLAGS
    Policy_Armed_Flag = false;
    Trg_Flag = false;
    onceFlag = false;

    // UPDATE COLLISION RADIUS
    Collision_Radius = L_eff;


    // RESET LOGGED TRIGGER VALUES
    Trg_Flag = false;
    Pos_B_O_trg = vzero();
    Vel_B_O_trg = vzero();
    Quat_B_O_trg = mkquat(0.0f,0.0f,0.0f,1.0f);
    Omega_B_O_trg = vzero();

    Pos_P_B_trg = vzero();
    Vel_B_P_trg = vzero();
    Quat_P_B_trg = mkquat(0.0f,0.0f,0.0f,1.0f);
    Omega_B_P_trg = vzero();

    D_perp_trg = 0.0f;
    D_perp_CR_trg = 0.0f;

    Theta_x_trg = 0.0f;
    Theta_y_trg = 0.0f;
    Tau_trg = 0.0f;
    Tau_CR_trg = 0.0f;

    Y_output_trg[0] = 0.0f;
    Y_output_trg[1] = 0.0f;
    Y_output_trg[2] = 0.0f;
    Y_output_trg[3] = 0.0f;

    a_Trg_trg = 0.0f;
    a_Rot_trg = 0.0f;

    // RESET LOGGED IMPACT VALUES
    Impact_Flag_OB = false;
    Impact_Flag_Ext = false;
    Vel_mag_B_P_impact_OB = 0.0f;
    Vel_angle_B_P_impact_OB = 0.0f;
    Quat_B_O_impact_OB = mkquat(0.0f,0.0f,0.0f,1.0f);
    Omega_B_O_impact_OB = vzero();
    dOmega_B_O_impact_OB = vzero();


    // TURN OFF IMPACT LEDS
    #ifdef CONFIG_SAR_EXP
    ledSet(LED_GREEN_L, 0);
    ledSet(LED_BLUE_NRF, 0);
    #endif

}


void controllerOutOfTree(control_t *control,const setpoint_t *setpoint, 
                                            const sensorData_t *sensors, 
                                            const state_t *state, 
                                            const uint32_t tick) 
{

    // CHECK FOR CRAZYSWARM SIGNAL
    #ifdef CONFIG_SAR_EXP
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick))
    {
        uint32_t now = xTaskGetTickCount();
        if (now - PrevCrazyswarmTick > 1000)
        {
            Armed_Flag = false;
        }
    }
    #endif

    // POLICY UPDATES
    if (isOFUpdated == true) {

        isOFUpdated = false;

        if(Policy_Armed_Flag == true){


            switch (Policy)
            {
                case PARAM_OPTIM:

                    // EXECUTE POLICY IF TRIGGERED
                    if(Tau_CR <= a_Trg && onceFlag == false && abs(Tau_CR) <= 3.0f && Tau_CR >= 0.05f){

                        onceFlag = true;

                        // UPDATE AND RECORD TRIGGER VALUES
                        Trg_Flag = true;  
                        Pos_B_O_trg = Pos_B_O;
                        Vel_B_O_trg = Vel_B_O;
                        Quat_B_O_trg = Quat_B_O;
                        Omega_B_O_trg = Omega_B_O;

                        Pos_P_B_trg = Pos_P_B;
                        Vel_B_P_trg = Vel_B_P;
                        Quat_P_B_trg = Quat_P_B;
                        Omega_B_P_trg = Omega_B_P;

                        Vel_mag_B_P_trg = Vel_mag_B_P;
                        Vel_angle_B_P_trg = Vel_angle_B_P;

                        Tau_trg = Tau;
                        Tau_CR_trg = Tau_CR;
                        Theta_x_trg = Theta_x;
                        Theta_y_trg = Theta_y;
                        D_perp_trg = D_perp;
                        D_perp_CR_trg = D_perp_CR;

                        a_Trg_trg = a_Trg;
                        a_Rot_trg = a_Rot;

                        M_d.x = 0.0f;
                        M_d.y = a_Rot*Iyy;
                        M_d.z = 0.0f;
                        }
                        
                    break;

                case DEEP_RL_SB3:

                    // EXECUTE POLICY IF TRIGGERED
                    if(onceFlag == false){

                        onceFlag = true;

                        // UPDATE AND RECORD TRIGGER VALUES
                        Trg_Flag = true;  
                        Pos_B_O_trg = Pos_B_O;
                        Vel_B_O_trg = Vel_B_O;
                        Quat_B_O_trg = Quat_B_O;
                        Omega_B_O_trg = Omega_B_O;

                        Pos_P_B_trg = Pos_P_B;
                        Vel_B_P_trg = Vel_B_P;
                        Quat_P_B_trg = Quat_P_B;
                        Omega_B_P_trg = Omega_B_P;

                        Tau_trg = Tau;
                        Tau_CR_trg = Tau_CR;
                        Theta_x_trg = Theta_x;
                        Theta_y_trg = Theta_y;
                        D_perp_trg = D_perp;
                        D_perp_CR_trg = D_perp_CR;


                        a_Trg_trg = a_Trg;
                        a_Rot_trg = a_Rot;

                        M_d.x = 0.0f;
                        M_d.y = a_Rot*Iyy;
                        M_d.z = 0.0f;
                    }

                    break;

                case DEEP_RL_ONBOARD:

                    // PASS OBSERVATION THROUGH POLICY NN
                    NN_forward(X_input,Y_output,&NN_DeepRL);

                    // printf("X_input: %.5f %.5f %.5f %.5f\n",X_input->data[0][0],X_input->data[1][0],X_input->data[2][0],X_input->data[3][0]);
                    // printf("Y_output: %.5f %.5f %.5f %.5f\n\n",Y_output->data[0][0],Y_output->data[1][0],Y_output->data[2][0],Y_output->data[3][0]);


                    // SAMPLE POLICY TRIGGER ACTION
                    a_Trg = GaussianSample(Y_output->data[0][0],Y_output->data[2][0]);
                    a_Rot = GaussianSample(Y_output->data[1][0],Y_output->data[3][0]);

                    // SCALE ACTIONS
                    a_Trg = scaleValue(tanhf(a_Trg),-1.0f,1.0f,-1.0f,1.0f);
                    a_Rot = scaleValue(tanhf(a_Rot),-1.0f,1.0f,a_Rot_bounds[0],a_Rot_bounds[1]);

                    // EXECUTE POLICY IF TRIGGERED
                    if(a_Trg >= 0.5f && onceFlag == false && abs(Tau_CR) <= 1.0f)
                    {
                        onceFlag = true;

                        // UPDATE AND RECORD TRIGGER VALUES
                        Trg_Flag = true;  
                        Pos_B_O_trg = Pos_B_O;
                        Vel_B_O_trg = Vel_B_O;
                        Quat_B_O_trg = Quat_B_O;
                        Omega_B_O_trg = Omega_B_O;

                        Pos_P_B_trg = Pos_P_B;
                        Vel_B_P_trg = Vel_B_P;
                        Quat_P_B_trg = Quat_P_B;
                        Omega_B_P_trg = Omega_B_P;

                        Tau_trg = Tau;
                        Tau_CR_trg = Tau_CR;
                        Theta_x_trg = Theta_x;
                        Theta_y_trg = Theta_y;
                        D_perp_trg = D_perp;
                        D_perp_CR_trg = D_perp_CR;

                        Y_output_trg[0] = Y_output->data[0][0];
                        Y_output_trg[1] = Y_output->data[1][0];
                        Y_output_trg[2] = Y_output->data[2][0];
                        Y_output_trg[3] = Y_output->data[3][0];

                        a_Trg_trg = a_Trg;
                        a_Rot_trg = a_Rot;

                        M_d.x = 0.0f;
                        M_d.y = a_Rot*Iyy;
                        M_d.z = 0.0f;
                    }
                        
                    break;

                
                    
            default:
                break;
            }

        }

    }


    if (RATE_DO_EXECUTE(RATE_25_HZ, tick))
    {
        updateRotationMatrices();
    }

    
    // STATE UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {

        float time_delta = (tick-prev_tick)/1000.0f;

        // CALC STATES WRT ORIGIN
        Pos_B_O = mkvec(state->position.x, state->position.y, state->position.z);               // [m]
        Vel_B_O = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);               // [m/s]
        Accel_B_O = mkvec(sensors->acc.x*9.81f, sensors->acc.y*9.81f, sensors->acc.z*9.81f);    // [m/s^2]
        Accel_B_O_Mag = firstOrderFilter(vmag(Accel_B_O),Accel_B_O_Mag,1.0f) - 9.81f;           // [m/s^2]

        Omega_B_O = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));   // [rad/s]

        // CALC AND FILTER ANGULAR ACCELERATION
        dOmega_B_O.x = firstOrderFilter((Omega_B_O.x - Omega_B_O_prev.x)/time_delta,dOmega_B_O.x,1.0f); // [rad/s^2]
        dOmega_B_O.y = firstOrderFilter((Omega_B_O.y - Omega_B_O_prev.y)/time_delta,dOmega_B_O.y,1.0f); // [rad/s^2]
        dOmega_B_O.z = firstOrderFilter((Omega_B_O.z - Omega_B_O_prev.z)/time_delta,dOmega_B_O.z,1.0f); // [rad/s^2]

        Quat_B_O = mkquat(state->attitudeQuaternion.x,
                        state->attitudeQuaternion.y,
                        state->attitudeQuaternion.z,
                        state->attitudeQuaternion.w);

        // CALC STATES WRT PLANE
        Pos_P_B = mvmul(R_WP,vsub(r_P_O,Pos_B_O)); 
        Vel_B_P = mvmul(R_WP,Vel_B_O);
        Vel_mag_B_P = vmag(Vel_B_P);
        Vel_angle_B_P = atan2f(Vel_B_P.z,Vel_B_P.x)*Rad2Deg;
        Omega_B_P = Omega_B_O;

        // LAGGING STATES TO RECORD IMPACT VALUES
        // NOTE: A circular buffer would be a true better option if time allows
        if (cycleCounter % 5 == 0)
        {
            Vel_mag_B_P_prev_N = Vel_mag_B_P;
            Vel_angle_B_P_prev_N = Vel_angle_B_P;
            Quat_B_O_prev_N = Quat_B_O;
            Omega_B_O_prev_N = Omega_B_O;
            dOmega_B_O_prev_N = dOmega_B_O;
        }
        cycleCounter++;
        

        // ONBOARD IMPACT DETECTION
        if (dOmega_B_O.y > 300.0f && Impact_Flag_OB == false)
        {
            Impact_Flag_OB = true;
            Vel_mag_B_P_impact_OB = Vel_mag_B_P_prev_N;
            Vel_angle_B_P_impact_OB = Vel_angle_B_P_prev_N;
            Quat_B_O_impact_OB = Quat_B_O_prev_N;
            Omega_B_O_impact_OB = Omega_B_O_prev_N;
            dOmega_B_O_impact_OB.y = dOmega_B_O_prev_N.y;

            // TURN ON IMPACT LEDS
            #ifdef CONFIG_SAR_EXP
            ledSet(LED_GREEN_R, 1);
            ledSet(LED_BLUE_NRF, 1);
            #endif
        }

        // SAVE PREVIOUS VALUES
        Omega_B_O_prev = Omega_B_O;
        prev_tick = tick;
    }

    // OPTICAL FLOW UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick))
    {
        // UPDATE GROUND TRUTH OPTICAL FLOW
        updateOpticalFlowAnalytic(state,sensors);

        // POLICY VECTOR UPDATE
        if (CamActive_Flag == true)
        {
            // ONLY UPDATE WITH NEW OPTICAL FLOW DATA
            // isOFUpdated = updateOpticalFlowEst();

            // UPDATE POLICY VECTOR
            // X_input->data[0][0] = Tau_Cam;
            // X_input->data[1][0] = Theta_x_Cam;
            // X_input->data[2][0] = D_perp; 
            // X_input->data[3][0] = Plane_Angle_deg; 
        }
        else
        {
            // UPDATE AT THE ABOVE FREQUENCY
            isOFUpdated = true;

            // UPDATE POLICY VECTOR
            X_input->data[0][0] = scaleValue(Tau_CR,-5.0f,5.0f,-1.0f,1.0f);
            X_input->data[1][0] = scaleValue(Theta_x,-20.0f,20.0f,-1.0f,1.0f);
            X_input->data[2][0] = scaleValue(D_perp_CR,-0.5f,2.0f,-1.0f,1.0f); 
            X_input->data[3][0] = scaleValue(Plane_Angle_deg,0.0f,180.0f,-1.0f,1.0f);
        }
    }
    
    // TRAJECTORY UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {

        switch (Traj_Type)
        {
            case NONE:
                /* DO NOTHING */
                break;

            case P2P:
                point2point_Traj();
                break;

            case CONST_VEL:
                const_velocity_Traj();
                break;

            case GZ_CONST_VEL:
                const_velocity_GZ_Traj();
                break;
        }
    }
        
    // CTRL UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {


        controlOutput(state,sensors);
        // consolePrintf("Thrust_max: %f\n",Thrust_max);
        F_thrust = clamp(F_thrust,0.0f,Thrust_max*g2Newton*4*0.85f);

        if(AngAccel_Flag == true || Trg_Flag == true)
        {
            F_thrust = 0.0f;
            M = vscl(2.0f,M_d);
        }

        // MOTOR MIXING (GTC_Derivation_V2.pdf) (Assume Symmetric Quadrotor)
        M1_thrust = (F_thrust - 1/Prop_14_y * M.x - 1/Prop_14_x * M.y - 1/C_tf * M.z) * 1/4.0f;
        M2_thrust = (F_thrust - 1/Prop_14_y * M.x + 1/Prop_14_x * M.y + 1/C_tf * M.z) * 1/4.0f;
        M3_thrust = (F_thrust + 1/Prop_14_y * M.x + 1/Prop_14_x * M.y - 1/C_tf * M.z) * 1/4.0f;
        M4_thrust = (F_thrust + 1/Prop_14_y * M.x - 1/Prop_14_x * M.y + 1/C_tf * M.z) * 1/4.0f;


        // CLAMP AND CONVERT THRUST FROM [N] AND [N*M] TO [g]
        M1_thrust = clamp(M1_thrust*Newton2g,0.0f,Thrust_max);
        M2_thrust = clamp(M2_thrust*Newton2g,0.0f,Thrust_max);
        M3_thrust = clamp(M3_thrust*Newton2g,0.0f,Thrust_max);
        M4_thrust = clamp(M4_thrust*Newton2g,0.0f,Thrust_max);

        // TUMBLE DETECTION
        if(b3.z <= 0 && TumbleDetect_Flag == true){ // If b3 axis has a negative z-component (Quadrotor is inverted)
            Tumbled_Flag = true;
        }

        
        if(CustomThrust_Flag) // REPLACE THRUST VALUES WITH CUSTOM VALUES
        {
            M1_thrust = thrust_override[0];
            M2_thrust = thrust_override[1];
            M3_thrust = thrust_override[2];
            M4_thrust = thrust_override[3];

            // CONVERT THRUSTS TO M_CMD SIGNALS
            M1_CMD = (int32_t)thrust2Motor_CMD(M1_thrust); 
            M2_CMD = (int32_t)thrust2Motor_CMD(M2_thrust);
            M3_CMD = (int32_t)thrust2Motor_CMD(M3_thrust);
            M4_CMD = (int32_t)thrust2Motor_CMD(M4_thrust);
        }
        else if(CustomMotorCMD_Flag)
        {
            M1_CMD = M_CMD_override[0]; 
            M2_CMD = M_CMD_override[1];
            M3_CMD = M_CMD_override[2];
            M4_CMD = M_CMD_override[3];
        }
        else 
        {
            // CONVERT THRUSTS TO M_CMD SIGNALS
            M1_CMD = (int32_t)thrust2Motor_CMD(M1_thrust); 
            M2_CMD = (int32_t)thrust2Motor_CMD(M2_thrust);
            M3_CMD = (int32_t)thrust2Motor_CMD(M3_thrust);
            M4_CMD = (int32_t)thrust2Motor_CMD(M4_thrust);
        }

        if (!Armed_Flag || MotorStop_Flag || Tumbled_Flag || Impact_Flag_OB || Impact_Flag_Ext)
        {
            #ifndef CONFIG_SAR_EXP
            M1_thrust = 0.0f;
            M2_thrust = 0.0f;
            M3_thrust = 0.0f;
            M4_thrust = 0.0f;
            #endif

            M1_CMD = 0.0f; 
            M2_CMD = 0.0f;
            M3_CMD = 0.0f;
            M4_CMD = 0.0f;
        }


        

        #ifdef CONFIG_SAR_EXP
        if(Armed_Flag)
        {
            // SEND CMD VALUES TO MOTORS
            motorsSetRatio(MOTOR_M1, M1_CMD);
            motorsSetRatio(MOTOR_M2, M2_CMD);
            motorsSetRatio(MOTOR_M3, M3_CMD);
            motorsSetRatio(MOTOR_M4, M4_CMD);
            
            // TURN ON ARMING LEDS
            ledSet(LED_BLUE_L, 1);
        }
        else{
            motorsSetRatio(MOTOR_M1, 0);
            motorsSetRatio(MOTOR_M2, 0);
            motorsSetRatio(MOTOR_M3, 0);
            motorsSetRatio(MOTOR_M4, 0);
            
            // TURN OFF ARMING LEDS
            ledSet(LED_BLUE_L, 0);
        }
        #endif

        // COMPRESS STATES
        compressStates();
        compressTrgStates();
        compressImpactOBStates();
        compressSetpoints();


    }


}


#ifdef CONFIG_SAR_EXP


// NOTE: PARAM GROUP + NAME + 1 CANNOT EXCEED 26 CHARACTERS (WHY? IDK.)
// NOTE: CANNOT HAVE A LOG AND A PARAM ACCESS THE SAME VALUE
PARAM_GROUP_START(P1)
PARAM_ADD(PARAM_FLOAT, Mass,        &m)
PARAM_ADD(PARAM_FLOAT, I_xx,        &Ixx)
PARAM_ADD(PARAM_FLOAT, I_yy,        &Iyy)
PARAM_ADD(PARAM_FLOAT, Izz,         &Izz)

PARAM_ADD(PARAM_FLOAT, Prop_14_x,   &Prop_14_x)
PARAM_ADD(PARAM_FLOAT, Prop_14_y,   &Prop_14_y)
PARAM_ADD(PARAM_FLOAT, Prop_23_x,   &Prop_23_x)
PARAM_ADD(PARAM_FLOAT, Prop_23_y,   &Prop_23_y)

PARAM_ADD(PARAM_FLOAT, C_tf,        &C_tf)
PARAM_ADD(PARAM_FLOAT, Tust_max,    &Thrust_max)
PARAM_ADD(PARAM_FLOAT, L_eff,       &L_eff)
PARAM_ADD(PARAM_FLOAT, Fwd_Reach,   &Forward_Reach)


PARAM_ADD(PARAM_UINT8, Armed,       &Armed_Flag)
PARAM_ADD(PARAM_UINT8, PolicyType,  &Policy)
PARAM_ADD(PARAM_UINT8, SAR_Type,    &SAR_Type)
PARAM_GROUP_STOP(P1)


PARAM_GROUP_START(P2)
PARAM_ADD(PARAM_FLOAT, P_kp_xy,         &P_kp_xy)
PARAM_ADD(PARAM_FLOAT, P_kd_xy,         &P_kd_xy) 
PARAM_ADD(PARAM_FLOAT, P_ki_xy,         &P_ki_xy)
PARAM_ADD(PARAM_FLOAT, i_range_xy,      &i_range_xy)

PARAM_ADD(PARAM_FLOAT, P_kp_z,          &P_kp_z)
PARAM_ADD(PARAM_FLOAT, P_Kd_z,          &P_kd_z)
PARAM_ADD(PARAM_FLOAT, P_ki_z,          &P_ki_z)
PARAM_ADD(PARAM_FLOAT, i_range_z,       &i_range_z)

PARAM_ADD(PARAM_FLOAT, R_kp_xy,         &R_kp_xy)
PARAM_ADD(PARAM_FLOAT, R_kd_xy,         &R_kd_xy) 
PARAM_ADD(PARAM_FLOAT, R_ki_xy,         &R_ki_xy)
PARAM_ADD(PARAM_FLOAT, i_range_R_xy,    &i_range_R_xy)

PARAM_ADD(PARAM_FLOAT, R_kpz,           &R_kp_z)
PARAM_ADD(PARAM_FLOAT, R_kdz,           &R_kd_z)
PARAM_ADD(PARAM_FLOAT, R_ki_z,          &R_ki_z)
PARAM_ADD(PARAM_FLOAT, i_range_R_z,     &i_range_R_z)
PARAM_GROUP_STOP(P2)



LOG_GROUP_START(Z_States)

LOG_ADD(LOG_UINT32, r_BO,           &States_Z.r_BOxy)
LOG_ADD(LOG_INT16,  r_BOz,          &States_Z.r_BOz)
LOG_ADD(LOG_UINT32, V_BOxy,         &States_Z.V_BOxy)
LOG_ADD(LOG_INT16,  V_BOz,          &States_Z.V_BOz)
LOG_ADD(LOG_UINT32, Acc_BOxy,       &States_Z.Acc_BOxy)
LOG_ADD(LOG_INT16,  Acc_BOz,        &States_Z.Acc_BOz)
LOG_ADD(LOG_UINT32, Quat_BO,        &States_Z.Quat_BO)
LOG_ADD(LOG_UINT32, Omega_BOxy,     &States_Z.Omega_BOxy)
LOG_ADD(LOG_INT16,  Omega_BOz,      &States_Z.Omega_BOz)
LOG_ADD(LOG_INT16,  dOmegaBOy,      &States_Z.dOmega_BOy)
LOG_ADD(LOG_INT16,  Acc_BO_Mag,     &States_Z.Accel_BO_Mag)

LOG_ADD(LOG_UINT32, VelRel_BP,      &States_Z.VelRel_BP)
LOG_ADD(LOG_UINT32, r_PBxy,         &States_Z.r_PBxy)
LOG_ADD(LOG_INT16,  r_PBz,          &States_Z.r_PBz)

LOG_ADD(LOG_UINT32, D_perp,         &States_Z.D_perp)
LOG_ADD(LOG_UINT32, Tau,            &States_Z.Tau)
LOG_ADD(LOG_INT16,  Theta_x,        &States_Z.Theta_x)
LOG_ADD(LOG_UINT32, Pol_Actions,    &States_Z.Policy_Actions)

LOG_ADD(LOG_UINT32, FMz,            &States_Z.FMz)
LOG_ADD(LOG_UINT32, Mxy,            &States_Z.Mxy)
LOG_ADD(LOG_UINT32, f_12,           &States_Z.M_thrust12)
LOG_ADD(LOG_UINT32, f_34,           &States_Z.M_thrust34)
LOG_ADD(LOG_UINT32, M_CMD12,        &States_Z.M_CMD12)
LOG_ADD(LOG_UINT32, M_CMD34,        &States_Z.M_CMD34)
LOG_GROUP_STOP(Z_States)


LOG_GROUP_START(Z_SetPoints)
LOG_ADD(LOG_UINT32, x_xy,           &SetPoints_Z.xy)
LOG_ADD(LOG_INT16,  x_z,            &SetPoints_Z.z)

LOG_ADD(LOG_UINT32, v_xy,           &SetPoints_Z.vxy)
LOG_ADD(LOG_INT16,  v_z,            &SetPoints_Z.vz)

LOG_ADD(LOG_UINT32, a_xy,           &SetPoints_Z.axy)
LOG_ADD(LOG_INT16,  a_z,            &SetPoints_Z.az)
LOG_GROUP_STOP(Z_SetPoints)



LOG_GROUP_START(Z_Trg)
LOG_ADD(LOG_UINT8,  Trg_Flag,       &Trg_Flag)
LOG_ADD(LOG_UINT32, r_BOxy,         &TrgStates_Z.r_BOxy)
LOG_ADD(LOG_INT16,  r_BOz,          &TrgStates_Z.r_BOz)
LOG_ADD(LOG_UINT32, V_BOxy,         &TrgStates_Z.V_BOxy)
LOG_ADD(LOG_INT16,  V_BOz,          &TrgStates_Z.V_BOz)
LOG_ADD(LOG_UINT32, Quat_BO,        &TrgStates_Z.Quat_BO)
LOG_ADD(LOG_INT16,  Omega_BOy,      &TrgStates_Z.Omega_BOy)
LOG_ADD(LOG_UINT32, VelRel_BP,      &TrgStates_Z.VelRel_BP)
LOG_ADD(LOG_UINT32, r_PBxy,         &TrgStates_Z.r_PBxy)
LOG_ADD(LOG_INT16,  r_PBz,          &TrgStates_Z.r_PBz)
LOG_ADD(LOG_UINT32, D_perp,         &TrgStates_Z.D_perp)
LOG_ADD(LOG_UINT32, Tau,            &TrgStates_Z.Tau)
LOG_ADD(LOG_INT16,  Theta_x,        &TrgStates_Z.Theta_x)
LOG_ADD(LOG_UINT32, Pol_Actions,    &TrgStates_Z.Policy_Actions)
LOG_GROUP_STOP(Z_Trg)


LOG_GROUP_START(Z_Impact)
LOG_ADD(LOG_UINT8,  ImpaOB,         &Impact_Flag_OB)
LOG_ADD(LOG_UINT32, VelRel_BP,      &ImpactOB_States_Z.VelRel_BP)
LOG_ADD(LOG_UINT32, Quat_BO,        &ImpactOB_States_Z.Quat_BO)
LOG_ADD(LOG_INT16,  OmegaBOy,       &ImpactOB_States_Z.Omega_BOy)
LOG_ADD(LOG_INT16,  dOmega_BOy,     &ImpactOB_States_Z.dOmega_BOy)
LOG_GROUP_STOP(Z_Impact)



LOG_GROUP_START(Misc)
LOG_ADD(LOG_FLOAT, Pos_Ctrl,        &kp_xf)
LOG_ADD(LOG_FLOAT, Vel_Ctrl,        &kd_xf)
LOG_ADD(LOG_UINT8, Motorstop,       &MotorStop_Flag)
LOG_ADD(LOG_UINT8, Tumbled_Flag,    &Tumbled_Flag)
LOG_ADD(LOG_UINT8, Tumble_Detect,   &TumbleDetect_Flag)
LOG_ADD(LOG_UINT8, AngAccel_Flag,   &AngAccel_Flag)
LOG_ADD(LOG_UINT8, Armed_Flag,      &Armed_Flag)
LOG_ADD(LOG_UINT8, Policy_Armed,    &Policy_Armed_Flag)
LOG_ADD(LOG_UINT8, CustomThrust,    &CustomThrust_Flag)
LOG_ADD(LOG_UINT8, CustomM_CMD,     &CustomMotorCMD_Flag)
LOG_ADD(LOG_FLOAT, r_PO_x,          &r_P_O.x)
LOG_ADD(LOG_FLOAT, r_PO_y,          &r_P_O.y)
LOG_ADD(LOG_FLOAT, r_PO_z,          &r_P_O.z)
LOG_ADD(LOG_FLOAT, Plane_Angle,     &Plane_Angle_deg)
LOG_ADD(LOG_UINT16,CMD_ID,          &CMD_ID)
LOG_GROUP_STOP(Misc)
#endif