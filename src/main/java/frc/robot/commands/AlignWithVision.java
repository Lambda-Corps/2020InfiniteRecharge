/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

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

  /**
   * Creates a new AlignWithVision.
   */
  public AlignWithVision(DriveTrain driveTrain, Vision vision) {
    m_DriveTrain = driveTrain;
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain, m_vision);

    SmartDashboard.putNumber("Steering KP", 0.055);  // TODO -- Tune KPs on carpet
    SmartDashboard.putNumber("min TA", 2.1);
    SmartDashboard.putNumber("Driving KP", 0.80);
  }

  // Called when the command is initially scheduled.S
  @Override
  public void initialize() {
    m_steeringKP = SmartDashboard.getNumber("Steering KP", 0.0);
    m_targetArea = SmartDashboard.getNumber("min TA", 0.0);
    m_driveKP = SmartDashboard.getNumber("Driving KP", 0.0);

    SmartDashboard.putNumber("Left", 10000);
    SmartDashboard.putNumber("Right", 10000);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double right = m_vision.getTX()*m_steeringKP; // Right Y
    double left  = (m_targetArea-m_vision.getTA())*m_driveKP; // Left X
    SmartDashboard.putNumber("target area", m_vision.getTA());

    // m_DriveTrain.teleop_drive(left, right); // Drive until the target is at desired distance
    SmartDashboard.putNumber("Left", left);
    SmartDashboard.putNumber("Right", right);
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
