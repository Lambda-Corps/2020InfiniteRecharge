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
   //Climbing Subsytem
    public static final int TOP_LIMIT_SWITCH     = 1;
    public static final int BOTTEM_LIMIT_SWITCH  = 0;
    public static final int CLIMBER_CHANNEL_A    = 2;
    public static final int CLIMBER_CHANNEL_B    = 3;
    // Talons
    public static final int CLIMBER_MOTOR          = 1;
    public static final int LEFT_TALON_LEADER      = 3;
    public static final int LEFT_TALON_FOLLOWER    = 4;
    public static final int RIGHT_TALON_LEADER     = 5;
    public static final int RIGHT_TALON_FOLLOWER   = 6;
    public static final int COLOR_WHEEL_TALON      = 7; // this is spinning the Motor
    public static final int INSIDE_INTAKE          = 10;   //This Talon controls the motor 
    public static final int OUTSIDE_INTAKE         = 11;       // This Motor 
    public static final int OUTSIDE_INTAKE_INDEXER = 12; //INTAKE THAT TAKES THE POWER CELLS UP TO THE SHOOTER
}