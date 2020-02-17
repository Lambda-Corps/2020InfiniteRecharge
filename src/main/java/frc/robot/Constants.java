/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Operator Interface 
    public static final int DRIVER_REMOTE_PORT   = 0;
    public static final int DRIVER_RIGHT_AXIS    = 4;
    public static final int DRIVER_LEFT_AXIS     = 1;
    public static final int PARTNER_REMOTE_PORT  = 1;
    public static final double CONTROLLER_DEADBAND_POSOTIVE = 0.1;
    public static final double CONTROLLER_DEADBAND_NEGATIVE = -0.1;
   //Climbing Subsytem
    public static final int TOP_LIMIT_SWITCH     = 1;
    public static final int BOTTEM_LIMIT_SWITCH  = 0;
    public static final int CLIMBER_CHANNEL_A    = 2;
    public static final int CLIMBER_CHANNEL_B    = 3;
    
    // Solenoids
    public static final int GEARBOX_SOLENOID_A = 0;
    public static final int GEARBOX_SOLENOID_B = 1;

    public static final int CLIMBER_MOTOR          = 1;
    public static final int LEFT_TALON_LEADER      = 3;
    public static final int LEFT_TALON_FOLLOWER    = 4;
    public static final int RIGHT_TALON_LEADER     = 5;
    public static final int RIGHT_TALON_FOLLOWER   = 6;
    public static final int COLOR_WHEEL_TALON      = 7; // this is spinning the Motor
    public static final int INSIDE_INTAKE          = 10;   //This Talon controls the motor 
    public static final int OUTSIDE_INTAKE         = 11;       // This Motor 
    public static final int OUTSIDE_INTAKE_INDEXER = 12; //INTAKE THAT TAKES THE POWER CELLS UP TO THE SHOOTER

    // DriveTrain Configuration Constants
    public static final double  DT_OPENLOOP_RAMP_RATE  = .3;
    public static final int     DT_CONTINUOUS_CURRENT  = 20;
    public static final double OPEN_LOOP_PEAK_OUTPUT_F = 1;
    public static final double OPEN_LOOP_PEAK_OUTPUT_B = -1;

    // DriveTrain Shifting Thresholds
    public static final int UP_SHIFT_SPEED = 1500;
    public static final int DOWN_SHIFT_SPEED = 1200;
    public static final double DT_FORWARD_L_MODIFIER = .951;
    public static final double DT_REVERSE_L_MODIFIER = .969;

    // Closed Loop
    /**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
     * Not all set of Gains are used in this project and may be removed as desired.
     * 
	 * 	                                    			  kP   kI   kD   kF               Iz    PeakOut */
	public final static Gains kGains_DriveMM = new Gains( 0.1, 0.0,  0.0, 0.0,            100,  0.50 );
	public final static Gains kGains_TurnMM = new Gains( 2.0, 0.0,  4.0, 0.0,            200,  1.00 );
	public final static Gains kGains_Shooter = new Gains( 0.1, 0.0, 20.0, 1023.0/6800.0,  300,  0.50 );
	public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/3200.0,  400,  1.00 );
	
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

}