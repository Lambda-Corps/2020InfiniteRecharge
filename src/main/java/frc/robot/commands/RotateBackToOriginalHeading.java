/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class RotateBackToOriginalHeading extends CommandBase {
  DriveTrain m_drivetrain;
  int m_arcLengthTicks;
  double m_arcLengthDegrees;
  int count_ok;
  //private double m_start_time;
  int STABLE_ITERATIONS_BEFORE_FINISHED = 5;
  //public final ShuffleboardTab turnMMTab;
  // private double m_turn_kP, m_kI, m_kD;
  // private NetworkTableEntry m_turnkPEntry, m_kIEntry, m_kDEntry, m_arclengthEntry, m_iterationEntry, m_drivedurationEntry, m_countokEntry, m_arclengthticksEntry;
  /**
   * Creates a new TurnMM.
   */
  public RotateBackToOriginalHeading(DriveTrain driveTrain) {
    m_drivetrain = driveTrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_drivetrain.disableMotorSafety();
    /*512 ticks per 1 rotation * 54/30 * 36/12 (gearing) = ~2764.8 ticks/1 wheel rotation (2765 is rounded)
      2765 ticks per 1 rotation / (pi * 6.3125 in (wheel diameter)) = 139.416 ticks per 1 inch
      Measured wheel base is 28 inches, so to figure out inches per degree of turning we take 
      Circumference / 360.
        28 * pi = 87.964594300514211 
        87.965 / 360 degrees = .2443 degrees per inch

      ArcLength in ticks = ticks per inch * degrees per inch * degrees to turn
    */
    // m_arcLengthDegrees = m_arclengthEntry.getDouble(0);
    m_arcLengthDegrees = -m_drivetrain.get_last_heading();
    m_arcLengthTicks = (int) (m_arcLengthDegrees * 139.416 * 0.2465); 
    count_ok = 0;
    m_drivetrain.reset_drivetrain_encoders();
    m_drivetrain.motion_magic_start_config_turn(m_arcLengthDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.feedWatchdog();
    if (m_drivetrain.motionMagicTurn(m_arcLengthTicks)){
      count_ok++;
    } else {
      count_ok = 0;
    }
    // m_countokEntry.setDouble(count_ok);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.motion_magic_end_config_turn();
    // double drive_duration = Timer.getFPGATimestamp() - m_start_time;
    // m_drivedurationEntry.setDouble(drive_duration);
    m_drivetrain.stopMotors();
    m_drivetrain.enableMotorSafety();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count_ok >= STABLE_ITERATIONS_BEFORE_FINISHED;
  }
}