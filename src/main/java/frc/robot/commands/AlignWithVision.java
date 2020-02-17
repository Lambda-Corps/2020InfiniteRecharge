/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class AlignWithVision extends CommandBase {
  private final DriveTrain m_DriveTrain;
  private final Vision m_vision;

  // Constants: tune driving and steering control constants
  private double m_steeringKP = 0.055;
  private double m_targetArea = 2.1;
  private double m_driveKP = 0.80;

  // Network Table Entries used for Tuning
  NetworkTableEntry m_kpSteer, m_minTA, m_drive_kp;
  /**
   * Creates a new AlignWithVision.
   */
  public AlignWithVision(DriveTrain driveTrain, Vision vision) {
    m_DriveTrain = driveTrain;
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain, m_vision);

    m_kpSteer = Shuffleboard.getTab("VisionAlign").add("Steering KP", 0.055).getEntry();
    m_minTA = Shuffleboard.getTab("VisionAlign").add("min TA", 2.1).getEntry();
    m_drive_kp = Shuffleboard.getTab("VisionAlign").add("Driving KP", 0.8).getEntry();
  }

  // Called when the command is initially scheduled.S
  @Override
  public void initialize() {
    m_steeringKP = m_kpSteer.getDouble(0);
    m_targetArea = m_minTA.getDouble(0);
    m_driveKP = m_drive_kp.getDouble(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double right = m_vision.getTX()*m_steeringKP; // Right Y
    double left  = (m_targetArea-m_vision.getTA())*m_driveKP; // Left X
    SmartDashboard.putNumber("target area", m_vision.getTA());

    // m_DriveTrain.teleop_drive(left, right); // Drive until the target is at desired distance
    Shuffleboard.getTab("VisionAlign").add("left calc", left).getEntry().forceSetDouble(left);
    Shuffleboard.getTab("VisionAlign").add("right calc", right).getEntry().forceSetDouble(right);
   
    if (m_vision.isTargetValid()) {
      if (m_vision.getTA() >= m_targetArea) {
        left = 0;
        right = 0;
      }
    } else {
     left = 0;
     right = 0;
    }

    m_DriveTrain.teleop_drive(left, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.teleop_drive(0, 0); // set left and right values to 0
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // don't return true ever, command will end after button is released
    return false;
  }
}
