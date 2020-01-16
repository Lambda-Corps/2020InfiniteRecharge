/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import static frc.robot.Constants.*;


public class Drive_Backwards extends CommandBase {
  private final DriveTrain m_driveTrain;
  private final XboxController m_driverController;
  /**
   * Creates a new Drive_Backwards.
   */
  public Drive_Backwards(DriveTrain driveTrain, XboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_driverController = driverController;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double right = m_driverController.getRawAxis(DRIVER_RIGHT_AXIS); // Right X
    double left  = -m_driverController.getRawAxis(DRIVER_LEFT_AXIS); // Left Y

    m_driveTrain.teleop_drive(-left, right);
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