/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShuffleboardInfo;
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

    m_kpSteer = ShuffleboardInfo.getInstance().getKPsteerEntry();
    m_minTA = ShuffleboardInfo.getInstance().getTargetEntry();
    m_drive_kp = ShuffleboardInfo.getInstance().getKPDriveEntry();
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
    double right = m_vision.getTX()*m_steeringKP; // Right X
    double left  = (m_targetArea-m_vision.getTA())*m_driveKP; // Left Y
   
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
