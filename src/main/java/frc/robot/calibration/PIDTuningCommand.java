/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibration;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/*
 * This command is designed to be used for any sort of PID calibration that
 * we need to do.  It can be used with a little tweaking to be MotionMagic,
 * MotionProfile, Velocity, or Position.
 * 
 * Feel free to copy the source file to make a new one, or to extend this one
 * and make it your own. This example is using the drive train for the base 
 * subsystem, but can be used with many subsystem by changing the argument type.
 * 
 * The command will use Shuffleboard to put all of it's values that will be used
 * to tune the pid itself.  Then, on robot init, this Command can be sent to the 
 * Dashboard as well for tuning.
 */
public class PIDTuningCommand extends CommandBase {
  private boolean isDone;

  double m_setpoint, m_tolerance;
  private DriveTrain m_dt;
  private final ShuffleboardTab m_myTab;
  private NetworkTableEntry m_kpEntry, m_kiEntry, m_kdEntry, m_kfEntry, m_spEntry, m_tolEntry, m_status;
  /**
   * Creates a new PIDTuningCommand.
   */
  public PIDTuningCommand( DriveTrain dt ) {
    // add all the Shuffleboard configuration values
    m_myTab = Shuffleboard.getTab("PID Tuning");

    m_dt = dt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_dt);
  }

  // This constructor exists ONLY for the reason to have it be testable in the Robot Simulator
  // The Robot Simulator doesn't have
  public PIDTuningCommand( ) {
    // add all the Shuffleboard configuration values
    m_myTab = Shuffleboard.getTab("PID Tuning");
    m_dt = null;

    m_kpEntry = m_myTab.add("kP", 0).withPosition(1, 3).getEntry();
    m_kiEntry = m_myTab.add("kI", 0 ).withPosition(2, 3).getEntry();
    m_kdEntry = m_myTab.add("kD", 0 ).withPosition(3, 3).getEntry();
    m_kfEntry = m_myTab.add("kF", 0 ).withPosition(0, 3).getEntry();
    m_spEntry = m_myTab.add("Set Point", 0 ).withPosition(4, 3).getEntry();
    m_tolEntry = m_myTab.add("Error Tolerance", 0 ).withPosition(5, 3).getEntry();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Setup the values for this first run:
    isDone = false;

    // Grab the member variables from Shuffleboard for this next run
    double kp = m_kpEntry.getDouble(0);
    double ki = m_kiEntry.getDouble(0);
    double kd = m_kdEntry.getDouble(0);
    double kf = m_kfEntry.getDouble(0);
    m_setpoint = m_spEntry.getDouble(0);
    m_tolerance = m_tolEntry.getDouble(0);

    System.out.println("Values kp: " + kp + " ki: " + ki );
    // Setup the PID configuration in the Talons
    if( m_dt != null){
      m_dt.setPIDValues(kp, ki, kd, kf);
    }  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run the PID control loop on the talon.  It's okay to call this repeatedly.
    // Constantly setting the setpoint in a position or velocity PID is fine.

    // In the case of PID controls, this call to start the actual PID loop could 
    // be done in initialize, because calling it once and calling it repeatedly 
    // without changing the PID or set point values is basically the same thing.
    if( m_dt != null ){
      isDone = m_dt.drivePositionPID(m_setpoint, m_tolerance);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Shut the motors off by driving in Percent
    if( m_dt != null ){
      m_dt.stopMotors();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}