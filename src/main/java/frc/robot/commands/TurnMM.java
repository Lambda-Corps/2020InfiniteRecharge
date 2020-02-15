/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnMM extends CommandBase {
  DriveTrain m_drivetrain;
  double m_arcLengthTicks, m_arcLengthDegrees;
  int count_ok;
  private double m_start_time;
  int STABLE_ITERATIONS_BEFORE_FINISHED = 5;
  public final ShuffleboardTab turnMMTab;
  private double m_turn_kP, m_kF, m_kI, m_kD;
  private NetworkTableEntry m_turnkPEntry, m_kFEntry, m_kIEntry, m_kDEntry, m_arclengthEntry, m_iterationEntry, m_drivedurationEntry, m_countokEntry;
  /**
   * Creates a new TurnMM.
   */
  public TurnMM(DriveTrain driveTrain, double angle) {
    m_drivetrain = driveTrain;
    addRequirements(m_drivetrain);
    turnMMTab = Shuffleboard.getTab("Turn MM Testing");
    m_turnkPEntry = turnMMTab.add("kP_turn", 0 ).withPosition(1, 0).getEntry();
    m_kIEntry = turnMMTab.add("kI", 0 ).withPosition(2, 0).getEntry();
    m_kDEntry = turnMMTab.add("kD", 0 ).withPosition(3, 0).getEntry();
    m_kFEntry = turnMMTab.add("kF", 0 ).withPosition(0, 0).getEntry();
    m_iterationEntry = turnMMTab.add("stable iteration before finishing", 5 ).withPosition(0, 1).getEntry();
    m_arclengthEntry = turnMMTab.add("target position", 0).withPosition(4, 0).getEntry();
    turnMMTab.addNumber("Left Encoder", m_drivetrain::getLeftEncoderValue).withPosition(1, 1);
    turnMMTab.addNumber("Right Encoder", m_drivetrain::getRightEncoderValue).withPosition(2,1);
    m_drivedurationEntry = turnMMTab.add("drive duration", 0).withPosition(6, 0).getEntry();
    m_countokEntry = turnMMTab.add("count_ok", 0).getEntry();

    this.m_arcLengthDegrees = angle;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*512 ticks per 1 rotation * 54/30 * 36/12 (gearing) = ~2764.8 ticks/1 wheel rotation (2765 is rounded)
      2765 ticks per 1 rotation / (pi * 6.3125 in (wheel diameter)) = 139.416 ticks per 1 inch*/
    this.m_arcLength = m_arcLength * 139.416 * 0.2443; 
    //TODO see if inches per angle needs tuned (got the number from last year's notes/code)
    count_ok = 0;
    m_drivetrain.reset_drivetrain_encoders();

    m_kF = m_kFEntry.getDouble(0.17785118219749652294853963838665);
    m_turn_kP = m_turnkPEntry.getDouble(0.0);
    m_kI = m_kIEntry.getDouble(0.0);
    m_kD = m_kDEntry.getDouble(0.0);
    STABLE_ITERATIONS_BEFORE_FINISHED = (int) m_iterationEntry.getDouble(5.0);
    m_arcLengthDegrees = m_arclengthEntry.getDouble(0.0);
    /*512 ticks per 1 rotation * 54/30 * 36/12 (gearing) = ~2764.8 ticks/1 wheel rotation (2765 is rounded)
      2765 ticks per 1 rotation / (pi * 6.3125 in (wheel diameter)) = 139.416 ticks per 1 inch*/
    this.m_arcLengthTicks = m_arcLengthDegrees * 139.416 * 0.2443; 
    //TODO see if inches per angle needs tuned (got the number from last year's notes/code)
    m_start_time = Timer.getFPGATimestamp();
    count_ok = 0;
    m_drivetrain.reset_drivetrain_encoders();
    m_drivetrain.reset_PID_values(m_turn_kP, m_kI, m_kD);
    m_drivetrain.motion_magic_start_config_turn(m_arcLengthDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.motionMagicTurn(m_arcLengthTicks);
    if (m_drivetrain.motionMagicOnTargetTurn(m_arcLengthTicks)){
      count_ok++;
    } else {
      count_ok = 0;
    }
    m_countokEntry.setDouble(count_ok);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.motion_magic_end_config_turn(m_arcLengthDegrees);
    double drive_duration = Timer.getFPGATimestamp() - m_start_time;
    m_drivedurationEntry.setDouble(drive_duration);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count_ok >= STABLE_ITERATIONS_BEFORE_FINISHED;
  }
}
