/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibration;

import java.util.function.DoubleSupplier;

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
  private double m_kp, m_ki, m_kd, m_kf, m_setpoint, m_tolerance;
  private boolean isDone;

  private final DriveTrain m_dt;
  private final ShuffleboardTab m_myTab;
  /**
   * Creates a new PIDTuningCommand.
   */
  public PIDTuningCommand( DriveTrain dt ) {
    // add all the Shuffleboard configuration values
    m_myTab = Shuffleboard.getTab("PID Tuning");

    // Setup each configuration value that we'll use from run to run.
    m_myTab.addNumber("kP", ()-> 0.0 );
    m_myTab.addNumber("kI", ()-> 0.0 );
    m_myTab.addNumber("kD", ()-> 0.0 );
    m_myTab.addNumber("kF", ()-> 0.0 );
    m_myTab.addNumber("Set Point", ()-> 0.0 );
    m_myTab.addNumber("Error Tolerance", ()->0.0);

    m_dt = dt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Setup the values for this first run:
    isDone = false;
    m_kp = m_myTab.add("kP",0).getEntry().getDouble(0.0);
    m_ki = m_myTab.add("kI",0).getEntry().getDouble(0.0);
    m_kd = m_myTab.add("kD",0).getEntry().getDouble(0.0);
    m_kf = m_myTab.add("kF",0).getEntry().getDouble(0.0);
    m_setpoint = m_myTab.add("Set Point",0).getEntry().getDouble(0.0);
    m_tolerance = m_myTab.add("Error Tolerance",0).getEntry().getDouble(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isDone = m_dt.drivePositionPID(m_setpoint, m_tolerance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Shut the motors off by driving in Percent
    m_dt.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
