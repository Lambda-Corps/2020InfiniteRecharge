/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnMM extends CommandBase {
  DriveTrain m_drivetrain;
  double m_arcLength;
  int count_ok;
  final int STABLE_ITERATIONS_BEFORE_FINISHED = 5;

  /**
   * Creates a new TurnMM.
   */
  public TurnMM(DriveTrain driveTrain, double angle) {
    m_drivetrain = driveTrain;
    addRequirements(m_drivetrain);

    this.m_arcLength = angle;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*512 ticks per 1 rotation * 54/30 * 36/12 (gearing) = ~2764.8 ticks/1 wheel rotation (2765 is rounded)
      2765 ticks per 1 rotation / (pi * 6.3125 in (wheel diameter)) = 139.416 ticks per 1 inch*/
    this.m_arcLength = m_arcLength * 139.416 * 0.2443; 
    //TODO see if inches per angle needs tuned (got the number from last year's notes/code)
    count_ok = 0;
    m_drivetrain.reset_drivetrain_encoders();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
