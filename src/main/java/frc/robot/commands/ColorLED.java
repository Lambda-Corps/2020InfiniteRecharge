/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDLights;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class ColorLED extends CommandBase {
  /**
   * Creates a new Color.
   */
  private Color White, Green, Blue, Red, Orange, Yellow;
  private final LEDLights m_Lights;
  private boolean isColor = false;

  public ColorLED() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Lights = new LEDLights();
    addRequirements(m_Lights);
    SmartDashboard.putData("What Color is Showing", m_Lights);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isColor = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooter
    // if shooting power cells show Orange lights
    // else if not shooting show green

    // intake
    // if it has zero balls show yellow
    // 1 ball show blue
    // 2 ball show yellow
    // 3 ball show red

    // drivetrain
    // high gear white
    // low gear purple

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isColor;
  }
}
