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
import static frc.robot.Constants.*;
public class RotateToTarget extends CommandBase {

  private final Vision m_vision;
  private final DriveTrain m_dt;
  private double m_steeringKP;
  private NetworkTableEntry m_steerKP;
  /**
   * Creates a new RotateToTTarget.
   */
  public RotateToTarget(Vision vision, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = vision;
    m_dt = driveTrain;
    addRequirements(vision, driveTrain);

    m_steerKP = ShuffleboardInfo.getInstance().getKPsteerEntry();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_steeringKP = m_steerKP.getDouble(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double right = 0;

    double tx = m_vision.getTX();
    if (tx < VISION_TX_TOLOERANCE && tx > -VISION_TX_TOLOERANCE) {
      right = 0;
    
    }else {
      right = tx * m_steeringKP;
    }
    // double right = m_vision.getTX()* m_steeringKP; 
    // if (right < .15 && )
    // // Right X

    m_dt.teleop_drive(0, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
