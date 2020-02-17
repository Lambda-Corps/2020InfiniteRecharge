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

public class DriveMM extends CommandBase {
  private final DriveTrain m_drivetrain;
  private double m_target_position;
  // private double m_target_multiplier;
  public int count_ok;
  private double m_start_time;
  public final ShuffleboardTab driveMMTab;
  private double m_drive_kP, m_kI, m_kD;
  private NetworkTableEntry m_drivekPEntry, m_kIEntry, m_kDEntry, m_targetPosEntry, m_iterationEntry, m_drivedurationEntry;
  //the number of times motion magic is on target before the command finishes
   int STABLE_ITERATIONS_BEFORE_FINISHED = 5;
  /**
   * Creates a new DriveMM.
   */
  public DriveMM(DriveTrain driveTrain, double goal_distance) {
    m_drivetrain = driveTrain;
    driveMMTab = Shuffleboard.getTab("Drive MM Testing");
    m_drivekPEntry = driveMMTab.add("kP_drive", 0 ).withPosition(1, 0).getEntry();
    m_kIEntry = driveMMTab.add("kI", 0 ).withPosition(2, 0).getEntry();
    m_kDEntry = driveMMTab.add("kD", 0 ).withPosition(3, 0).getEntry();
    //m_kFEntry = driveMMTab.add("kF", 0 ).withPosition(0, 0).getEntry();
    m_iterationEntry = driveMMTab.add("stable iteration before finishing", 5 ).withPosition(0, 1).getEntry();
    m_targetPosEntry = driveMMTab.add("target position", 0).withPosition(4, 0).getEntry();
   // m_targetMultEntry = driveMMTab.add("target multiplier", 0).withPosition(5, 0).getEntry();
    driveMMTab.addNumber("Left Encoder", m_drivetrain::getLeftEncoderValue).withPosition(1, 1);
    driveMMTab.addNumber("Right Encoder", m_drivetrain::getRightEncoderValue).withPosition(2,1);
    m_drivedurationEntry = driveMMTab.add("drive duration", 0).withPosition(6, 0).getEntry();
  
    addRequirements(m_drivetrain);
    //super("DriveMM", 5);
    this.m_target_position = goal_distance;
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_kF = m_kFEntry.getDouble(0.17785118219749652294853963838665);
    m_drive_kP = m_drivekPEntry.getDouble(0.0);
    m_kI =m_kIEntry.getDouble(0.0);
    m_kD = m_kDEntry.getDouble(0.0);
    STABLE_ITERATIONS_BEFORE_FINISHED = (int) m_iterationEntry.getDouble(5.0);
    m_target_position = m_targetPosEntry.getDouble(0.0);
    //m_target_multiplier = m_targetMultEntry.getDouble(0.0);

    m_start_time = Timer.getFPGATimestamp();
    /*512 ticks per 1 rotation * 54/30 * 36/12 (gearing) = ~2764.8 ticks/1 wheel rotation (2765 is rounded)
      2765 ticks per 1 rotation / (pi * 6.3125 in (wheel diameter)) = 139.416 ticks per 1 inch*/
    this.m_target_position = m_target_position * 139.416;
    count_ok = 0;
    m_drivetrain.reset_drivetrain_encoders();
    m_drivetrain.reset_drive_PID_values(m_drive_kP, m_kI, m_kD);
    m_drivetrain.motion_magic_start_config_drive();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_drivetrain.motionMagicDrive(m_target_position)){
      count_ok++;
    } else {
      count_ok = 0;
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.motion_magic_end_config_drive();
    double drive_duration = Timer.getFPGATimestamp() - m_start_time;
    m_drivedurationEntry.setDouble(drive_duration);
    m_drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count_ok >= STABLE_ITERATIONS_BEFORE_FINISHED;
  }
}
