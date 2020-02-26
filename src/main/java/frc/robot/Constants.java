/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Operator Interface 
    public static final int DRIVER_REMOTE_PORT   = 0;
    public static final int DRIVER_RIGHT_AXIS    = 4;
    public static final int DRIVER_LEFT_AXIS     = 1;
    public static final int PARTNER_REMOTE_PORT  = 1;
    public static final double CONTROLLER_DEADBAND_POSOTIVE = 0.1;
    public static final double CONTROLLER_DEADBAND_NEGATIVE = -0.1;
   //DIO
    public static final int TOP_LIMIT_SWITCH     = 1;
    public static final int BOTTEM_LIMIT_SWITCH  = 0;
    public static final int BOTTOM_LIMIT_SWITCH  = 0;
    public static final int BEAM_BREAKER_SEND    = 4;
    public static final int BEAM_BREAKER_RECEIVER_TOP   = 3;
    public static final int BEAM_BREAKER_RECEIVER_MIDDLETOP = 5;
    public static final int BEAM_BREAKER_RECEIVER_MIDDLEBOTTOM    = 6;
    public static final int BEAM_BREAKER_RECEIVER_BOTTOM    = 7;
    
    // Solenoids
    public static final int GEARBOX_SOLENOID_A = 0;
    public static final int GEARBOX_SOLENOID_B = 1;
    public static final int CLIMBER_CHANNEL_A    = 2;
    public static final int CLIMBER_CHANNEL_B    = 3;
    public static final int INTAKE_SOLENOID_A  = 4;
    public static final int INTAKE_SOLENOID_B  = 5;
    public static final int SHOOTER_CHANNEL_A = 6;
    public static final int SHOOTER_CHANNEL_B = 7;

    // Talons
    public static final int CLIMBER_MOTOR          = 1;
    public static final int INTAKE_CONVEYOR        = 2;
    public static final int LEFT_TALON_LEADER      = 3;
    public static final int LEFT_TALON_FOLLOWER    = 4;
    public static final int RIGHT_TALON_LEADER     = 5;
    public static final int RIGHT_TALON_FOLLOWER   = 6;
    public static final int COLOR_WHEEL_TALON      = 7; // this is spinning the Motor
    public static final int BOTTOM_SHOOTER         = 8;
    public static final int TOP_SHOOTER            = 9;
    public static final int INTAKE                 = 10;   //This Talon controls the motor 
    public static final int INTAKE_INDEXER         = 11;       // This Motor 
    public static final int UNUSED                 = 12; 

    // DriveTrain Configuration Constants
    public static final double  DT_OPENLOOP_RAMP_RATE  = .3;
    public static final int     DT_CONTINUOUS_CURRENT  = 20;
    public static final double OPEN_LOOP_PEAK_OUTPUT_F = 1;
    public static final double OPEN_LOOP_PEAK_OUTPUT_B = -1;

    // DriveTrain Shifting Thresholds
    public static final int UP_SHIFT_SPEED = 1300;
    public static final int DOWN_SHIFT_SPEED = 1000;
    public static final double DT_FORWARD_L_MODIFIER = .959;
    public static final double DT_REVERSE_L_MODIFIER = .974;
    public static final int DT_MOTION_CRUISE_VEL = 2100;
    public static final int DT_MOTION_ACCELERATION = 500;
    public static final Value DT_LOW_GEAR = Value.kForward;
    public static final Value DT_HIGH_GEAR = Value.kReverse;

    // Closed Loop
    /**
     * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
     * Not all set of Gains are used in this project and may be removed as desired.
     * 
     *                                                    kP   kI   kD   kF               Iz    PeakOut */
    public final static Gains kGains_DriveMM = new Gains( 4.75, 0.0, 0.0, 0.177850,          100,  1.00 );
    public final static Gains kGains_TurnMM_small = new Gains( 4.75, 0.0,  20, 0.09590625,   200,  1.00 );
    public final static Gains kGains_TurnMM_big   = new Gains( 0.85, 0.0,  20, 0.09590625,   200,  1.00 );
    public final static Gains kGains_Shooter = new Gains( 0.1, 0.0, 20.0, 1023.0/6800.0,  300,  0.50 );
    public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/3200.0,  400,  1.00 );
    public final static int kTimeoutMs = 5;

    /** ---- Flat constants, you should not need to change these ---- */
    /* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
    public final static int REMOTE_0 = 0;
    public final static int REMOTE_1 = 1;
    /* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
    public final static int PID_PRIMARY = 0;
    public final static int PID_AUXILIARY = 1;
    /* Firmware currently supports slots [0, 3] and can be used for either PID Set */
    public final static int SLOT_0 = 0;
    public final static int SLOT_1 = 1;
    public final static int SLOT_2 = 2;
    public final static int SLOT_3 = 3;
    public static final int DT_SLOT_DRIVE_MM = SLOT_0;
    public static final int DT_SLOT_AUXILIARY_PID = SLOT_1;
    public static final int DT_SLOT_TURN_MM = SLOT_2;
    public static final int DT_SLOT_MOTION_PROFILE = SLOT_3;

    //Color wheel
    public static final int SPINNING_THREE_TIMES_SPEED = 1;
    public static final int SPINNING_THREE_TIMES_COUNTER = 450;
    public static final double SPINNING_TO_A_COLOR = 0.5;

    // Climber Constants
    public static final double CLIMBER_UP_SPEED = 1.0;
    public static final double CLIMBER_DOWN_SPEED = -1.0;
    public static final double CLIMBER_RAMP_RATE = 1.5;
    public static final int    CLIMBER_CURRENT_LIMIT = 25;

    //shooter
    public static final int SHOOTER_SETPOINT_WALL = 9000;
    public static final int SHOOTER_SETPOINT_LINE = 9000;
    public static final int SHOOTER_SETPOINT_TRENCH = 9000;
    public static final double SHOOTER_RAMP_TIME = 1.0;
    public static final int RUNTIME = 5;
    // Each shooting distance may have different gains to make it work
    public static final int SHOOTER_SLOT_INITIATION_LINE = 0;
    public static final int SHOOTER_SLOT_PORTWALL        = 1;
    public static final int SHOOTER_SLOT_FRONT_TRENCH    = 2;
    public static final Gains kGains_InitiationLine = new Gains(0,0,0,0,100, .5);
    public static final Gains kGains_PortWall = new Gains(0,0,0,0,200, .5);
    public static final Gains kGains_FrontTrench = new Gains(0,0,0,0,300, .5);
    public static final Value SHOOTER_WALL_SHOT = Value.kForward;
    public static final Value SHOOTER_FAR_SHOT = Value.kReverse;

   //Intake
   public static int CONVEYOR_SPEED = 1;
   public static double INTAKE_SPEED = -.7;
   public static double INDEXER_SPEED = .7;  
   public static final Value INTAKE_UP_POSITION = Value.kReverse;
   public static final Value INTAKE_DOWN_POSITION = Value.kForward;
}