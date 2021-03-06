/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.SPINNING_THREE_TIMES_COUNTER;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheel;


public class Spin3Times extends CommandBase {
  /**
   * Creates a new Spin3Times.
  
   */
  private final ColorWheel m_subsystem;
  private boolean isDone = false;
  private double SpinThree = SPINNING_THREE_TIMES_COUNTER;

  public Spin3Times(ColorWheel subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    addRequirements(subsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isDone = false;
    SpinThree = SPINNING_THREE_TIMES_COUNTER;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.startSpinningThree();

    if (SpinThree-- <= 0) {
      this.isDone = true;

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopSpinning();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isDone;
  }
}
