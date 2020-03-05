/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.MIN_TURN_OUTPUT;
import static frc.robot.Constants.VISION_STEER_KP;
import static frc.robot.Constants.VISION_TX_TOLOERANCE;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
public class RotateToTarget extends CommandBase {

  private final Vision m_vision;
  private final DriveTrain m_dt;
  private int m_stable_done;

  /**
   * Creates a new RotateToTTarget.
   */
  public RotateToTarget(Vision vision, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = vision;
    m_dt = driveTrain;
    addRequirements(vision, driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stable_done = 0;
    m_dt.reset_gyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double right = 0;

    double tx = m_vision.getTX();
    if (tx < VISION_TX_TOLOERANCE && tx > -VISION_TX_TOLOERANCE) {
      right = 0;
      m_stable_done++;
    }else {
      m_stable_done = 0;
      right = tx * VISION_STEER_KP;
      if (right > 0 && right < MIN_TURN_OUTPUT) {
        right = MIN_TURN_OUTPUT;
      } else if (right < 0 && right > -MIN_TURN_OUTPUT) {
        right = -MIN_TURN_OUTPUT;
      }
    }

    m_dt.teleop_drive(0, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dt.set_last_heading();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_stable_done >= 20;
  }
}
