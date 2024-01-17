#include "Controller_GTC.h"




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
    ledSet(LED_BLUE_L, 0);
    ledSet(LED_BLUE_NRF, 0);
    #endif

    controllerOutOfTreeReset();
    controllerOutOfTreeTest();

    // INIT DEEP RL NN POLICY
    X_input = nml_mat_new(3,1);
    Y_output = nml_mat_new(4,1);

    // INIT DEEP RL NN POLICY
    // NN_init(&NN_DeepRL,NN_Params_DeepRL);

    consolePrintf("GTC Controller Initiated\n");
}


void controllerOutOfTreeReset() {

    consolePrintf("GTC Controller Reset\n");
    consolePrintf("Policy_Type: %d\n",Policy);
    J = mdiag(Ixx,Iyy,Izz);

    // RESET INTEGRATION ERRORS
    e_PI = vzero();
    e_RI = vzero();

    // TURN POS/VEL CONTROLLER FLAGS ON
    kp_xf = 1.0f;
    kd_xf = 1.0f;

    // RESET SETPOINTS TO HOME POSITION
    x_d = mkvec(0.0f,0.0f,0.4f);
    v_d = mkvec(0.0f,0.0f,0.0f);
    a_d = mkvec(0.0f,0.0f,0.0f);
    b1_d = mkvec(1.0f,0.0f,0.0f);

    // RESET SYSTEM FLAGS
    tumbled = false;
    motorstop_flag = false;
    customThrust_flag = false;
    customPWM_flag = false;
    moment_flag = false;

    // RESET TRAJECTORY FLAGS
    Traj_Type = NONE;

    // RESET POLICY FLAGS
    policy_armed_flag = false;
    flip_flag = false;
    onceFlag = false;


    // RESET LOGGED FLIP VALUES
    statePos_trg = vzero();
    stateVel_trg = vzero();
    stateAcc_trg = vzero();
    stateQuat_trg = mkquat(0.0f,0.0f,0.0f,1.0f);
    stateOmega_trg = vzero();
    state_dOmega_trg = vzero();

    Tau_trg = 0.0f;
    Theta_x_trg = 0.0f;
    Theta_y_trg = 0.0f;
    D_perp_trg = 0.0f;

    Policy_Trg_Action_trg = 0.0f;
    Policy_Flip_Action_trg = 0.0f;


    updatePlaneNormal(Plane_Angle);

}


void controllerOutOfTree(control_t *control,const setpoint_t *setpoint, 
                                            const sensorData_t *sensors, 
                                            const state_t *state, 
                                            const uint32_t tick) 
{
    // STATE UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {

        statePos = mkvec(state->position.x, state->position.y, state->position.z);          // [m]
        stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);          // [m/s]
        stateAcc = mkvec(sensors->acc.x*9.81f, sensors->acc.y*9.81f, sensors->acc.z*9.81f); // [m/s^2]
        AccMag = vmag(stateAcc);


        stateOmega = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));   // [rad/s]
        state_dOmega =  vdiv(vsub(stateOmega,stateOmega_prev),(tick-prev_tick)/1000.0f); // [rad/s^2]
        stateQuat = mkquat(state->attitudeQuaternion.x,
                        state->attitudeQuaternion.y,
                        state->attitudeQuaternion.z,
                        state->attitudeQuaternion.w);

        // EULER ANGLES EXPRESSED IN YZX NOTATION
        stateEul = quat2eul(stateQuat);
        stateEul.x = degrees(stateEul.x);
        stateEul.y = degrees(stateEul.y);
        stateEul.z = degrees(stateEul.z);

        // SAVE PREVIOUS VALUES
        stateOmega_prev = stateOmega;
        prev_tick = tick;
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

            case CONST_VEL_GZ:
                const_velocity_GZ_Traj();
                break;
        }
    }

    // OPTICAL FLOW UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick))
    {
        // UPDATE GROUND TRUTH OPTICAL FLOW
        updateOpticalFlowAnalytic(state,sensors);

        // POLICY VECTOR UPDATE
        if (CamActive == true)
        {
            // ONLY UPDATE WITH NEW OPTICAL FLOW DATA
            isOFUpdated = updateOpticalFlowEst();

            // UPDATE POLICY VECTOR
            X_input->data[0][0] = Tau_est;
            X_input->data[1][0] = Theta_x_est;
            X_input->data[2][0] = D_perp; 

        }
        else
        {
            // UPDATE AT THE ABOVE FREQUENCY
            isOFUpdated = true;

            // UPDATE POLICY VECTOR
            X_input->data[0][0] = Tau;
            X_input->data[1][0] = Theta_x;
            X_input->data[2][0] = D_perp; 
        }
    }
    
    // POLICY UPDATES
    if (isOFUpdated == true) {

        isOFUpdated = false;

        if(policy_armed_flag == true){

            switch (Policy)
            {
                case PARAM_OPTIM:

                    // EXECUTE POLICY IF TRIGGERED
                    if(Tau <= Policy_Trg_Action && onceFlag == false && V_perp > 0.1f){

                        onceFlag = true;

                        // UPDATE AND RECORD FLIP VALUES
                        flip_flag = true;  
                        statePos_trg = statePos;
                        stateVel_trg = stateVel;
                        stateAcc_trg = stateAcc;
                        stateQuat_trg = stateQuat;
                        stateOmega_trg = stateOmega;
                        state_dOmega_trg = state_dOmega;

                        Tau_trg = Tau;
                        Theta_x_trg = Theta_x_trg;
                        Theta_y_trg = Theta_y_trg;
                        D_perp_trg = D_perp;

                    
                        M_d.x = 0.0f;
                        M_d.y = Policy_Flip_Action*Iyy;
                        M_d.z = 0.0f;

                        F_thrust_flip = 0.0;
                        M_x_flip = M_d.x*1e3f;
                        M_y_flip = M_d.y*1e3f;
                        M_z_flip = M_d.z*1e3f;
                        }
                        
                    break;

                case DEEP_RL_ONBOARD:

                    // PASS OBSERVATION THROUGH POLICY NN
                    NN_forward(X_input,Y_output,&NN_DeepRL);

                    // SAMPLE POLICY TRIGGER ACTION
                    Policy_Trg_Action = GaussianSample(Y_output->data[0][0],exp(Y_output->data[2][0]));

                    // EXECUTE POLICY
                    if(Policy_Trg_Action >= Policy_Flip_threshold && onceFlag == false && V_perp > 0.1f){

                        onceFlag = true;

                        // SAMPLE AND SCALE BODY FLIP ACTION
                        Policy_Flip_Action = GaussianSample(Y_output->data[1][0],exp(Y_output->data[3][0]));
                        Policy_Flip_Action = scale_tanhAction(Policy_Flip_Action,ACTION_MIN,ACTION_MAX);

                        // UPDATE AND RECORD FLIP VALUES
                        flip_flag = true;  
                        statePos_trg = statePos;
                        stateVel_trg = stateVel;
                        stateAcc_trg = stateAcc;
                        stateQuat_trg = stateQuat;
                        stateOmega_trg = stateOmega;
                        state_dOmega_trg = state_dOmega;

                        Tau_trg = Tau;
                        Theta_x_trg = Theta_x_trg;
                        Theta_y_trg = Theta_y_trg;
                        D_perp_trg = D_perp;

                    
                        M_d.x = 0.0f;
                        M_d.y = -Policy_Flip_Action*1e-3f;
                        M_d.z = 0.0f;

                        F_thrust_flip = 0.0;
                        M_x_flip = M_d.x*1e3f;
                        M_y_flip = M_d.y*1e3f;
                        M_z_flip = M_d.z*1e3f;
                        }
                        
                    break;

                case DEEP_RL_SB3:

                    break;
                    
            default:
                break;
            }

        }

    }
    
    // CTRL UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {


        controlOutput(state,sensors);

        if(moment_flag == true || flip_flag == true)
        {
            F_thrust = 0.0f;
            M = M_d;
        }


        // MOTOR MIXING (GTC_Derivation_V2.pdf) 
        M1_thrust = F_thrust * Prop_23_x/(Prop_14_x + Prop_23_x) - M.x * 1/(Prop_14_y + Prop_23_y) - M.y * 1/(Prop_14_x + Prop_23_x) - M.z * Prop_23_y/(C_tf*(Prop_14_y + Prop_23_y));
        M2_thrust = F_thrust * Prop_14_x/(Prop_14_x + Prop_23_x) - M.x * 1/(Prop_14_y + Prop_23_y) + M.y * 1/(Prop_14_x + Prop_23_x) + M.z * Prop_14_y/(C_tf*(Prop_14_y + Prop_23_y));
        M3_thrust = F_thrust * Prop_14_x/(Prop_14_x + Prop_23_x) + M.x * 1/(Prop_14_y + Prop_23_y) + M.y * 1/(Prop_14_x + Prop_23_x) - M.z * Prop_14_y/(C_tf*(Prop_14_y + Prop_23_y));
        M4_thrust = F_thrust * Prop_23_x/(Prop_14_x + Prop_23_x) + M.x * 1/(Prop_14_y + Prop_23_y) - M.y * 1/(Prop_14_x + Prop_23_x) + M.z * Prop_23_y/(C_tf*(Prop_14_y + Prop_23_y));

        // CLAMP AND CONVER THRUST FROM [N] AND [N*M] TO [g]
        M1_thrust = clamp((M1_thrust/2.0f)*Newton2g,0.0f,f_max);
        M2_thrust = clamp((M2_thrust/2.0f)*Newton2g,0.0f,f_max);
        M3_thrust = clamp((M3_thrust/2.0f)*Newton2g,0.0f,f_max);
        M4_thrust = clamp((M4_thrust/2.0f)*Newton2g,0.0f,f_max);


        // TUMBLE DETECTION
        if(b3.z <= 0 && tumble_detection == true){ // If b3 axis has a negative z-component (Quadrotor is inverted)
            tumbled = true;
        }


        // UPDATE THRUST COMMANDS
        if(motorstop_flag || tumbled) // STOP MOTOR COMMANDS
        { 
            M1_thrust = 0.0f;
            M2_thrust = 0.0f;
            M3_thrust = 0.0f;
            M4_thrust = 0.0f;

        }
        else if(customThrust_flag) // REPLACE THRUST VALUES WITH CUSTOM VALUES
        {
            
            M1_thrust = thrust_override[0];
            M2_thrust = thrust_override[1];
            M3_thrust = thrust_override[2];
            M4_thrust = thrust_override[3];

        }

        // UPDATE PWM COMMANDS
        if(customPWM_flag)
        {
            M1_pwm = PWM_override[0]; 
            M2_pwm = PWM_override[1];
            M3_pwm = PWM_override[2];
            M4_pwm = PWM_override[3];
        }
        else 
        {
            // CONVERT THRUSTS TO PWM SIGNALS
            M1_pwm = (int32_t)thrust2PWM(M1_thrust); 
            M2_pwm = (int32_t)thrust2PWM(M2_thrust);
            M3_pwm = (int32_t)thrust2PWM(M3_thrust);
            M4_pwm = (int32_t)thrust2PWM(M4_thrust);
        }

        // COMPRESS STATES
        compressStates();
        compressSetpoints();
        compressTrgStates();

        #ifdef CONFIG_SAR_EXP
        if(safeModeEnable)
        {
            motorsSetRatio(MOTOR_M1, 0);
            motorsSetRatio(MOTOR_M2, 0);
            motorsSetRatio(MOTOR_M3, 0);
            motorsSetRatio(MOTOR_M4, 0);
        }
        else{
            // SEND PWM VALUES TO MOTORS
            motorsSetRatio(MOTOR_M1, M4_pwm);
            motorsSetRatio(MOTOR_M2, M3_pwm);
            motorsSetRatio(MOTOR_M3, M2_pwm);
            motorsSetRatio(MOTOR_M4, M1_pwm);
        }
        #endif


    }
 

}


#ifdef CONFIG_SAR_EXP


// NOTE: PARAM GROUP + NAME + 1 CANNOT EXCEED 26 CHARACTERS (WHY? IDK.)
// NOTE: CANNOT HAVE A LOG AND A PARAM ACCESS THE SAME VALUE
PARAM_GROUP_START(System_Params)
PARAM_ADD(PARAM_FLOAT, Mass, &m)
PARAM_ADD(PARAM_FLOAT, Ixx, &Ixx)
PARAM_ADD(PARAM_FLOAT, Iyy, &Iyy)
PARAM_ADD(PARAM_FLOAT, Izz, &Izz)

PARAM_ADD(PARAM_FLOAT, Prop_14_x, &Prop_14_x)
PARAM_ADD(PARAM_FLOAT, Prop_14_y, &Prop_14_y)
PARAM_ADD(PARAM_FLOAT, Prop_23_x, &Prop_23_x)
PARAM_ADD(PARAM_FLOAT, Prop_23_y, &Prop_23_y)

PARAM_ADD(PARAM_FLOAT, C_tf, &C_tf)
PARAM_ADD(PARAM_FLOAT, f_max, &f_max)

PARAM_ADD(PARAM_UINT8, SafeMode, &safeModeEnable)
PARAM_ADD(PARAM_UINT8, PolicyType, &Policy)
PARAM_ADD(PARAM_UINT8, CamActive, &CamActive)
PARAM_GROUP_STOP(System_Params)


PARAM_GROUP_START(CTRL_Params)
PARAM_ADD(PARAM_FLOAT, P_kp_xy, &P_kp_xy)
PARAM_ADD(PARAM_FLOAT, P_kd_xy, &P_kd_xy) 
PARAM_ADD(PARAM_FLOAT, P_ki_xy, &P_ki_xy)
PARAM_ADD(PARAM_FLOAT, i_range_xy, &i_range_xy)

PARAM_ADD(PARAM_FLOAT, P_kp_z,  &P_kp_z)
PARAM_ADD(PARAM_FLOAT, P_kd_z,  &P_kd_z)
PARAM_ADD(PARAM_FLOAT, P_ki_z,  &P_ki_z)
PARAM_ADD(PARAM_FLOAT, i_range_z, &i_range_z)

PARAM_ADD(PARAM_FLOAT, R_kp_xy, &R_kp_xy)
PARAM_ADD(PARAM_FLOAT, R_kd_xy, &R_kd_xy) 
PARAM_ADD(PARAM_FLOAT, R_ki_xy, &R_ki_xy)
PARAM_ADD(PARAM_FLOAT, i_range_R_xy, &i_range_R_xy)

PARAM_ADD(PARAM_FLOAT, R_kp_z,  &R_kp_z)
PARAM_ADD(PARAM_FLOAT, R_kd_z,  &R_kd_z)
PARAM_ADD(PARAM_FLOAT, R_ki_z,  &R_ki_z)
PARAM_ADD(PARAM_FLOAT, i_range_R_z, &i_range_R_z)
PARAM_GROUP_STOP(CTRL_Params)



LOG_GROUP_START(Z_States)
LOG_ADD(LOG_UINT32, xy,         &States_Z.xy)
LOG_ADD(LOG_INT16,  z,          &States_Z.z)

LOG_ADD(LOG_UINT32, vxy,        &States_Z.vxy)
LOG_ADD(LOG_INT16,  vz,         &States_Z.vz)

LOG_ADD(LOG_UINT32, quat,       &States_Z.quat)

LOG_ADD(LOG_UINT32, wxy,        &States_Z.wxy)
LOG_ADD(LOG_INT16,  wz,         &States_Z.wz)

LOG_ADD(LOG_UINT32, Thetaxy,    &States_Z.Theta_xy)
LOG_ADD(LOG_INT16,  Tau,        &States_Z.Tau)
LOG_ADD(LOG_INT16,  D_perp,     &States_Z.D_perp)

LOG_ADD(LOG_UINT32, FMz,        &States_Z.FMz)
LOG_ADD(LOG_UINT32, Mxy,        &States_Z.Mxy)

LOG_ADD(LOG_UINT32, f_12,       &States_Z.M_thrust12)
LOG_ADD(LOG_UINT32, f_34,       &States_Z.M_thrust34)

LOG_ADD(LOG_UINT32, PWM12,      &States_Z.MS_PWM12)
LOG_ADD(LOG_UINT32, PWM34,      &States_Z.MS_PWM34)
LOG_GROUP_STOP(Z_States)



LOG_GROUP_START(Z_Policy)
LOG_ADD(LOG_UINT32, Thetaxy_est,    &States_Z.Theta_xy_est)
LOG_ADD(LOG_INT16,  Tau_est,        &States_Z.Tau_est)
LOG_ADD(LOG_UINT32, Pol_Actions,    &States_Z.Policy_Actions)
LOG_GROUP_STOP(Z_Policy)



LOG_GROUP_START(Z_SetPoints)
LOG_ADD(LOG_UINT32, xy,         &SetPoints_Z.xy)
LOG_ADD(LOG_INT16,  z,          &SetPoints_Z.z)

LOG_ADD(LOG_UINT32, vxy,        &SetPoints_Z.vxy)
LOG_ADD(LOG_INT16,  vz,         &SetPoints_Z.vz)

LOG_ADD(LOG_UINT32, axy,        &SetPoints_Z.axy)
LOG_ADD(LOG_INT16,  az,         &SetPoints_Z.az)
LOG_GROUP_STOP(Z_SetPoints)



LOG_GROUP_START(Z_TrgStates)
LOG_ADD(LOG_UINT32, xy,             &TrgStates_Z.xy)
LOG_ADD(LOG_INT16,  z,              &TrgStates_Z.z)
LOG_ADD(LOG_UINT32, vxy,            &TrgStates_Z.vxy)
LOG_ADD(LOG_INT16,  vz,             &TrgStates_Z.vz)

LOG_ADD(LOG_UINT32, quat,           &TrgStates_Z.quat)

LOG_ADD(LOG_UINT32, wxy,            &TrgStates_Z.wxy)
LOG_ADD(LOG_INT16,  wz,             &TrgStates_Z.wz)

LOG_ADD(LOG_UINT32, Thetaxy,        &TrgStates_Z.Theta_xy)
LOG_ADD(LOG_INT16,  Tau,            &TrgStates_Z.Tau)
LOG_ADD(LOG_INT16,  D_perp,         &TrgStates_Z.D_perp)

LOG_ADD(LOG_UINT32, Thetaxy_est,    &TrgStates_Z.Theta_xy_est)
LOG_ADD(LOG_INT16,  Tau_est,        &TrgStates_Z.Tau_est)

LOG_ADD(LOG_UINT32, PolActions,    &TrgStates_Z.Policy_Actions)

LOG_ADD(LOG_UINT8, Flip_Flag, &flip_flag)
LOG_GROUP_STOP(Z_TrgStates)



LOG_GROUP_START(CTRL_Flags)
LOG_ADD(LOG_FLOAT, Pos_Ctrl,        &kp_xf)
LOG_ADD(LOG_FLOAT, Vel_Ctrl,        &kd_xf)
LOG_ADD(LOG_UINT8, Motorstop,       &motorstop_flag)
LOG_ADD(LOG_UINT8, Tumbled,         &tumbled)
LOG_ADD(LOG_UINT8, Tumble_Detect,   &tumble_detection)
LOG_ADD(LOG_UINT8, Moment_Flag,     &moment_flag)
LOG_ADD(LOG_UINT8, CamActive,     &CamActive)
LOG_ADD(LOG_UINT8, SafeModeEnable,  &safeModeEnable)
LOG_ADD(LOG_UINT8, Pol_Armed,       &policy_armed_flag)
LOG_ADD(LOG_UINT8, CustomThrust,    &customThrust_flag)
LOG_ADD(LOG_UINT8, CustomPWM,       &customPWM_flag)
LOG_ADD(LOG_UINT8, AttCtrl_Flag,    &attCtrlEnable)
LOG_GROUP_STOP(CTRL_Flags)
#endif