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
  boolean m_isDone;
  

  /**
   * Creates a new IntakeUp.
   */
  public ToggleIntake(Intake intake) {
    m_intake = intake;
    m_isDone = false;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake_state = m_intake.getIntakeState();
    if( m_intake_state == DeployState.DEPLOY ){
      m_intake.pullIntakeUp();
    }
    else if( m_intake_state == DeployState.STOW ){
      m_intake.putIntakeDown();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake_state = m_intake.getIntakeState();
    if( m_intake_state == DeployState.DEPLOY ){
      m_isDone = m_intake.pullInBalls();
    }
    else{
      m_isDone = true;
    }
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopMotors();
  }

  // Returns returns true automatically after settting the states
  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}
