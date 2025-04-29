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
    global roll_error_integral, roll_error_derivative, previous_roll_error, pos_error ,pitch_error_integral, pitch_error_derivative, previous_pitch_error
    global r_w1, wid_w1, x_body, y_body, z_body, omega, domega, dv, v, v_max, v_min, omega_max, omega_min, curr_vel_left, curr_vel_right
    global claw_prism_vel, claw_vel_max, claw_vel_min, claw_joint_vel,x_set_p, y_set_p, set_flag
    global manip_script, Imax, Imin, xref, chassis, initial_x, initial_y, curr_pos
    global position_control_active, position_control_initialized , pos_vel, keypress
      
    left_motor_handle = sim.getObject('/body/left_joint')
    right_motor_handle = sim.getObject('/body/right_joint')
    body_handle = sim.getObject('/body')
    left_wheel_handle = sim.getObject('/body/left_joint/left_wheel')
    right_wheel_handle = sim.getObject('/body/right_joint/right_wheel')
    claw_joint = sim.getObject('/body/ForceSensor/arm_base/arm_joint/grip_base/Prismatic_joint')
    claw_joint_ud = sim.getObject('/body/ForceSensor/arm_base/arm_joint')
    chassis = sim.getObject('/body/SBR_Assembly_7')
    
    sim.setObjectInt32Param(left_motor_handle, sim.jointintparam_dynctrlmode, sim.jointdynctrl_velocity)
    sim.setObjectInt32Param(right_motor_handle, sim.jointintparam_dynctrlmode, sim.jointdynctrl_velocity)
 
    sim.setJointTargetVelocity(left_motor_handle, 0)
    sim.setJointTargetVelocity(right_motor_handle, 0)
    
    manip_script = sim.getScript(sim.scripttype_simulation,'manipulator_script')
    start_time = sim.getSimulationTime()
    end_time = start_time + 30 
 
    roll_error_integral = 0.0
    roll_error_derivative = 0.0
    previous_roll_error = 0.0
    
    pitch_error_integral = 0.0
    pitch_error_derivative = 0.0
    previous_pitch_error = 0.0
 
    v_max = 0.4
    v_min = -0.4
    omega_min = -1.0
    omega_max = 1.0
    omega = 0
    v = 0
    claw_prism_vel = 0
    claw_joint_vel = 0
    claw_vel_max = 0.8
    claw_vel_min = -0.8

    initial_x = None
    initial_y = None
    pos_error = 0
    set_flag = 0
    Imax = 100
    Imin = -100
    xref = 0
    pos_vel = 0
    
    position_control_initialized = False
    position_control_active = False
    ##################################
 
def sysCall_actuation():
    # put your actuation code here
    # This function will be executed at each simulation time step
 
    ####### ADD YOUR CODE HERE ######
    # Hint: Use the error feedback and apply control algorithm here
    #       Provide the resulting actuation as input to the actuator joint
    global left_motor_handle, right_motor_handle,claw_joint,claw_joint_ud,body_orientation, Kp, Ki, Kd, roll_error_integral, roll_error_derivative, previous_roll_error
    global end_time
    global roll_error_integral, roll_error_derivative, previous_roll_error, set_flag, pos_error, x_set_p, y_set_p
    global r_w1, wid_w1, x_body, y_body, z_body, omega, domega,pos_dv, dv, v, v_max, v_min, omega_max, omega_min, claw_prism_vel, claw_vel_max, claw_vel_min, claw_joint_vel
    global pitch_error_integral, pitch_error_derivative, previous_pitch_error, initial_x, initial_y, pos_vel
    global position_control_active, position_control_initialized, pos_vel, desired_x, desired_y, keypress     

    Kp = 180
    Ki = 8
    Kd = 2
    
    roll_error, keypress, pitch_error = sysCall_sensing()
    current_time = sim.getSimulationTime()
    t_l_b = 20 - current_time
    #print("20-t = %f" %t_l_b)
    dt = sim.getSimulationTimeStep()
    xref = sim.getObjectPosition(chassis, -1)

    roll_error_integral += roll_error 
    roll_error_derivative = (roll_error - previous_roll_error) 
    control_signal_roll = Kp * roll_error + Ki * roll_error_integral + Kd * roll_error_derivative 
    
    pitch_error_integral += pitch_error 
    pitch_error_derivative = (pitch_error - previous_pitch_error) 
    control_signal_pitch = Kp * pitch_error + Ki * pitch_error_integral + Kd * pitch_error_derivative 
    
    if(roll_error_integral > Imax):
        roll_error_integral = Imax
       
    if(roll_error_integral < Imin):
        roll_error_integral = Imin
        
    if(pitch_error_integral > Imax):
        pitch_error_integral = Imax
       
    if(pitch_error_integral < Imin):
        pitch_error_integral = Imin
 
    dv = 0.13
    domega = 1.0
    pos_dv = -10
    
    if keypress is not None:
        if keypress == 2007:
            v -= dv
        if keypress == 2008:  
            v += dv
        if keypress == 2009:  
            omega += domega
        if keypress == 2010:  
            omega -= domega
        if keypress == 113: # q
            claw_prism_vel = 0.1
        if keypress == 101: # e
            claw_prism_vel = -0.1
        if keypress == 119: # w
            claw_joint_vel = -0.8
        if keypress == 115: # s
            claw_joint_vel = 0.8
        if keypress == 97:  # a
            sim.callScriptFunction("sysCall_actuation", manip_script)
        if keypress == 100: # d
            sim.callScriptFunction("sysCall_actuation", manip_script)
        if keypress == 111:
            position_control_active = 1
            position_control_initialized = 0
            
        if keypress == 112:
            position_control_active = 0
            position_control_initialized = 0
            
            
    if position_control_active == 1:
        if position_control_initialized == 1:
            # Use the initial position as the desired position
            desired_x = initial_x
            desired_y = initial_y
        else:
            # Initialize the desired position with the current position
            pos = sim.getObjectPosition(body_handle, -1)
            initial_x = pos[0]
            initial_y = pos[1]
            desired_x = initial_x
            desired_y = initial_y
            position_control_initialized = 1
        
        curr_pos = sim.getObjectPosition(chassis, -1)
        curr_x = curr_pos[0]
        curr_y = curr_pos[1]
        pos_error = math.sqrt((desired_x - curr_x)**2 + (desired_y - curr_y)**2)

        
        if desired_x - curr_x < 0:
            pos_vel = pos_error * 100
            print("pos vel = %f" %pos_vel)
            
        if desired_x - curr_x > 0:
            pos_vel = pos_error * -100
            print("pos vel = %f" %pos_vel)

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
    
    #if set_flag != 1:
        #if (v < 0.1 or v> -0.1) and current_time > 4:
            #body_position = sim.getObjectPosition(body_handle, -1)
            #x_set_p = body_position[0]
            #print("x set point = %f" %x_set_p)
            #y_set_p = body_position[1]
            #set_flag = 1
            
    #if x_set_p != None or y_set_p != None:
        #body_position_new = sim.getObjectPosition(body_handle, -1)
        #x_pos_error = x_set_p - body_position_new[0]
        #y_pos_error = y_set_p - body_position_new[1]
        #pos_error = math.sqrt(x_pos_error**2 + y_pos_error**2)
        #print("position error = %f" %pos_error)

    left_velocity = control_signal_roll + v_drive_l + control_signal_pitch + pos_vel
    right_velocity = control_signal_roll + v_drive_r + control_signal_pitch + pos_vel
 
    sim.setJointTargetVelocity(left_motor_handle, left_velocity)
    sim.setJointTargetVelocity(right_motor_handle, right_velocity)
    
    sim.setJointTargetVelocity(claw_joint, claw_prism_vel)
    sim.setJointTargetVelocity(claw_joint_ud, claw_joint_vel)

    #fin_vel_r = sim.getJointTargetVelocity(right_motor_handle)
    #print("velocity right = %f" %fin_vel_r)
    #fin_vel_l = sim.getJointTargetVelocity(left_motor_handle)
    #print("velocity left = %f" %fin_vel_l)
    

    previous_roll_error = roll_error
    previous_pitch_error = pitch_error
    omega = 0
    claw_prism_vel = 0
    claw_joint_vel = 0
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
    global pitch_error_integral, pitch_error_derivative, previous_pitch_error, xref, xerr, chassis, keypress
    
    inc = sim.getObjectOrientation(body_handle, -1)
    curr_vel_left = sim.getJointTargetVelocity(left_motor_handle)
    curr_vel_right = sim.getJointTargetVelocity(right_motor_handle)

    yaw, pitch, roll = sim.alphaBetaGammaToYawPitchRoll(inc[0], inc[1], inc[2])
 
    set_point = 0
    roll_error = set_point - roll 
    pitch_error = set_point - pitch 
    #print("error = %f" %error)
            
    ############### Keyboard Input ##############
    message,data,data2 = sim.getSimulatorMessage()
 
    if (message == sim.message_keypress):
        print("key = %f" %data[0])
        if data[0] == 2007: # forward
            return roll_error, 2007, pitch_error#,  position_error
 
        if data[0] == 2008: # back
            return roll_error, 2008, pitch_error#, position_error
 
        if data[0] == 2009: # left arrow
            return roll_error, 2009, pitch_error#, position_error
 
        if data[0] == 2010: # right arrow
            return roll_error, 2010, pitch_error#, position_error
            
        if data[0] == 113: # Q gripper close
            return roll_error, 113, pitch_error#, position_error
            
        if data[0] == 101: # E gripper open
            return roll_error, 101, pitch_error#, position_error
        
        if data[0] == 119: # W gripper up
            return roll_error, 119, pitch_error#, position_error
            
        if data[0] == 115: # S gripper down
            return roll_error, 115, pitch_error#, position_error
            
        if data[0] == 97: # A attach
            return roll_error, 97, pitch_error
            
        if data[0] == 100: # D detach
            return roll_error, 100, pitch_error
            
        if keypress == 111: #r position control on
            print("ON")
            return roll_error, 111, pitch_error
            
            
        if keypress == 112: #t position control off
            print("OFF")
            return roll_error, 112, pitch_error
            
        #print("unrecognized keypress")
    return roll_error, None, pitch_error#, position_error
 
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