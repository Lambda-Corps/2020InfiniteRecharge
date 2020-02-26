/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.DeployState;


public class ToggleIntake extends CommandBase {
  private Intake m_intake;
  private DeployState m_intake_state;
  

  /**
   * Creates a new IntakeUp.
   */
  public ToggleIntake(Intake intake) {
    m_intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake_state = m_intake.getIntakeState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if( m_intake_state == DeployState.DEPLOY ){
      m_intake.pullIntakeUp();
      m_intake.stopMotors();
    }
    else if( m_intake_state == DeployState.STOW ){
      m_intake.putIntakeDown();
      m_intake.pullInBalls();
    }
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if( interrupted ){
      m_intake.stopMotors();
    }
  }

  // Returns returns true automatically after settting the states
  @Override
  public boolean isFinished() {
    return true;
  }
}
