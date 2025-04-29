import time
import math
 
def sysCall_init():
    sim = require('sim')
 
    # do some initialization here
    # This function will be executed once when the simulation starts
 
    # Instead of using globals, you can do e.g.:
    # self.myVariable = 21000000
 
    ######## ADD YOUR CODE HERE #######
    # Hint: Initialize the scene objects which you will require
    #       Initialize algorithm related variables here
    print("Simulation initialized")
    global left_motor_handle, right_motor_handle, body_handle, left_wheel_handle, right_wheel_handle, claw_joint, claw_joint_ud
    global start_time, end_time
    global error_integral, error_derivative, previous_error, pos_error
    global r_w1, wid_w1, x_body, y_body, z_body, omega, domega, dv, v, v_max, v_min, omega_max, omega_min, curr_vel_left, curr_vel_right
    global claw_prism_vel, claw_vel_max, claw_vel_min, claw_joint_vel,x_set_p, y_set_p, set_flag
    global manip_script
 
    left_motor_handle = sim.getObject('/body/left_joint')
    right_motor_handle = sim.getObject('/body/right_joint')
    body_handle = sim.getObject('/body')
    left_wheel_handle = sim.getObject('/body/left_joint/left_wheel')
    right_wheel_handle = sim.getObject('/body/right_joint/right_wheel')
    claw_joint = sim.getObject('/body/ForceSensor/arm_base/arm_joint/grip_base/Prismatic_joint')
    claw_joint_ud = sim.getObject('/body/ForceSensor/arm_base/arm_joint')
    sim.setObjectInt32Param(left_motor_handle, sim.jointintparam_dynctrlmode, sim.jointdynctrl_velocity)
    sim.setObjectInt32Param(right_motor_handle, sim.jointintparam_dynctrlmode, sim.jointdynctrl_velocity)
 
    sim.setJointTargetVelocity(left_motor_handle, 0)
    sim.setJointTargetVelocity(right_motor_handle, 0)
 
    manip_script = sim.getScript(sim.scripttype_simulation,'manipulator_script')
    start_time = sim.getSimulationTime()
    end_time = start_time + 30 
 
    error_integral = 0.0
    error_derivative = 0.0
    previous_error = 0.0
 
    v_max = 0.4
    v_min = -0.4
    omega_min = -0.5
    omega_max = 0.5
    omega = 0
    v = 0
    claw_prism_vel = 0
    claw_joint_vel = 0
    claw_vel_max = 0.8
    claw_vel_min = -0.8
 
    x_set_p = None
    y_set_p = None
    pos_error = 0
    set_flag = 0
    ##################################
 
def sysCall_actuation():
    # put your actuation code here
    # This function will be executed at each simulation time step
 
    ####### ADD YOUR CODE HERE ######
    # Hint: Use the error feedback and apply control algorithm here
    #       Provide the resulting actuation as input to the actuator joint
    global left_motor_handle, right_motor_handle,claw_joint,claw_joint_ud,body_orientation, Kp, Ki, Kd, error_integral, error_derivative, previous_error
    global end_time
    global error_integral, error_derivative, previous_error, set_flag, pos_error, x_set_p, y_set_p
    global r_w1, wid_w1, x_body, y_body, z_body, omega, domega,pos_dv, dv, v, v_max, v_min, omega_max, omega_min, claw_prism_vel, claw_vel_max, claw_vel_min, claw_joint_vel
 
 
    Kp = 1200/2
    Ki = 0.3/2
    Kd = 2/2
 
    error, keypress = sysCall_sensing()
    current_time = sim.getSimulationTime()
    t_l_b = 20 - current_time
    #print("20-t = %f" %t_l_b)
    dt = sim.getSimulationTimeStep()
    error_integral += error 
    error_derivative = (error - previous_error) 
    control_signal = Kp * error + Ki * error_integral + Kd * error_derivative 
 
    dv = 0.3
    domega = 0.5
    pos_dv = -10
 
    if keypress is not None:
        if keypress == 2007: 
            v -= dv
        elif keypress == 2008:  
            v += dv
        elif keypress == 2009:  
            omega -= domega
        elif keypress == 2010:  
            omega += domega
        elif keypress == 113: # q
            claw_prism_vel = 0.1
        elif keypress == 101: # e
            claw_prism_vel = -0.1
        elif keypress == 119: # w
            claw_joint_vel = -0.8
        elif keypress == 115: # s
            claw_joint_vel = 0.8
        elif keypress == 97:  # a
            sim.callScriptFunction("sysCall_actuation", manip_script)
        elif keypress == 100: # d
            sim.callScriptFunction("sysCall_actuation", manip_script)
 
    if omega != 0:
            if (omega>omega_max):
                omega = omega_max;
            if (omega<omega_min):
                omega = omega_min
 
    if v != 0:
           if (v>v_max):
               v=v_max
           if (v<v_min):
               v=v_min
 
    if claw_prism_vel != 0:
           if (claw_prism_vel>claw_vel_max):
               claw_prism_vel=claw_vel_max
           if (claw_prism_vel<claw_vel_min):
               claw_prism_vel=claw-vel_min
 
    if claw_joint_vel != 0:
           if (claw_joint_vel>claw_vel_max):
               claw_joint_vel=claw_vel_max
           if (claw_joint_vel<claw_vel_min):
               claw_joint_vel=claw-vel_min
 
    print("v = %f" %v)
    #print("omega = %f" %omega) 
    r = 0.0085 #radius of wheel
    b = 0.07 #half width, width is distance between the wheels
 
    v_drive_l = (v/r)-(omega*b/r)
    v_drive_r = (v/r)+(omega*b/r)
 
    if set_flag != 1:
        if (v < 0.1 or v> -0.1) and current_time > 4:
            body_position = sim.getObjectPosition(body_handle, -1)
            x_set_p = body_position[0]
            print("x set point = %f" %x_set_p)
            y_set_p = body_position[1]
            set_flag = 1
 
    if x_set_p != None or y_set_p != None:
        body_position_new = sim.getObjectPosition(body_handle, -1)
        x_pos_error = x_set_p - body_position_new[0]
        y_pos_error = y_set_p - body_position_new[1]
        pos_error = math.sqrt(x_pos_error**2 + y_pos_error**2)
        print("position error = %f" %pos_error)
 
    pos_vel = pos_error * pos_dv
    left_velocity = control_signal + v_drive_l + pos_vel
    right_velocity = control_signal + v_drive_r + pos_vel
 
    sim.setJointTargetVelocity(left_motor_handle, left_velocity)
    sim.setJointTargetVelocity(right_motor_handle, right_velocity)
 
    sim.setJointTargetVelocity(claw_joint, claw_prism_vel)
    sim.setJointTargetVelocity(claw_joint_ud, claw_joint_vel)
 
    #fin_vel_r = sim.getJointTargetVelocity(right_motor_handle)
    #print("velocity right = %f" %fin_vel_r)
    #fin_vel_l = sim.getJointTargetVelocity(left_motor_handle)
    #print("velocity left = %f" %fin_vel_l)
 
 
    previous_error = error
    omega = 0
    claw_prism_vel = 0
    claw_joint_vel = 0
 
    if sim.getSimulationTime() > end_time:
        sim.stopSimulation()
    # Example psuedo code:
    #   x1 = error_state_1; # Error in states w.r.t desired setpoint
    #   x2 = error_state_2;
    #   x3 = error_state_3;
    #   x4 = error_state_4;
    #   k = [gain_1 , gain_1, gain_3, gain_4];      # These gains will be generated by control algorithm. For ex: LQR, PID, etc.
    #   U = -k[1]*x1 +k[2]*x2 -k[3]*x3 +k[4]*x4;    # +/- Sign convention may differ according to implementation
    #   Set_joint_actuation(U);                     # Provide this calculated input to system's actuator
 
    #################################
 
def sysCall_sensing():
    # put your sensing code here
    # This function will be executed at each simulation time step
 
    ####### ADD YOUR CODE HERE ######
    # Hint: Take feedback here & do the error calculation
    global body_position, body_orientation, roll, pitch, yaw, body_position_new, curr_vel_left, curr_vel_right, y_set_p, x_set_p
    global x_pos_error,y_pos_error, position_error, x_set_p, y_set_p, dv
 
    inc = sim.getObjectOrientation(body_handle, -1)
    curr_vel_left = sim.getJointTargetVelocity(left_motor_handle)
    curr_vel_right = sim.getJointTargetVelocity(right_motor_handle)
 
    yaw, pitch, roll = sim.alphaBetaGammaToYawPitchRoll(inc[0], inc[1], inc[2])
 
    set_point = 0
    error = set_point - roll
    #print("error = %f" %error)
 
    ############### Keyboard Input ##############
    message,data,data2 = sim.getSimulatorMessage()
 
    if (message == sim.message_keypress):
        if data[0] == 2007: # forward
            return error, 2007#,  position_error
 
        if data[0] == 2008: # back
            return error, 2008#, position_error
 
 
        if data[0] == 2009: # left arrow
            return error, 2009#, position_error
 
 
        if data[0] == 2010: # right arrow
            return error, 2010#, position_error
 
        if data[0] == 113: # Q gripper close
            return error, 113#, position_error
 
        if data[0] == 101: # E gripper open
            return error, 101#, position_error
 
        if data[0] == 119: # W gripper up
            return error, 119#, position_error
 
        if data[0] == 115: # S gripper down
            return error, 115#, position_error
 
        if data[0] == 97: # A attach
            return error, 97
 
        if data[0] == 100: # D detach
            return error, 100
 
    else:
        #print("unrecognized keypress")
        return error, None#, position_error
 
    #########################################
 
def sysCall_cleanup():
    # do some clean-up here
    # This function will be executed when the simulation ends
 
    ####### ADD YOUR CODE HERE ######
    pass
    # Any cleanup (if required) to take the scene back to it's original state after simulation
    # It helps in case simulation fails in an unwanted state.
    #################################
 
 
# See the user manual or the available code snippets for additional callback functions and details