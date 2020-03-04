/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class HoldIntakeDown extends CommandBase {
  private final Intake m_Intake;
  private boolean m_IsDone;
  
  /**
   * Creates a new HoldIntakeDown.
   */
  public HoldIntakeDown(Intake intake) {
    m_Intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.putIntakeDown();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IsDone = m_Intake.pullInBalls();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.stopMotors();
    m_Intake.pullIntakeUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IsDone;
  }
}
