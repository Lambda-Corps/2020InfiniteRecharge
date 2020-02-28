/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class DefaultIntakeCommand extends CommandBase {
  private final Intake m_intake;
  private final XboxController m_partner_controller;
  /**
   * Creates a new DefaultIntakeCommand.
   */
  public DefaultIntakeCommand(Intake intake, XboxController controller) {
    m_intake = intake;
    m_partner_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Values should be from 0 - 1 on the triggers

    double rightTrigger = m_partner_controller.getRawAxis(XboxController.Axis.kRightTrigger.value);
    if( rightTrigger > .5 ){
      m_intake.EjectBalls();
    }
    else{
       m_intake.stopMotors();
    }
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
