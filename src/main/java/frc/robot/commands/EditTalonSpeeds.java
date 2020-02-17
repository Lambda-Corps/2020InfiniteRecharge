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

public class EditTalonSpeeds extends CommandBase {
  private final DriveTrain m_driveTrain; 
  private int m_cycleCounter;

  private double left, right;
  /**
   * Creates a new EditTalonSpeeds.
   */
  public EditTalonSpeeds(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
    SmartDashboard.putNumber("Cycles to run", 150);
    SmartDashboard.putNumber("Cycles ran", 0);
    SmartDashboard.putNumber("Left Speed", 0);
    SmartDashboard.putNumber("Right Speed", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.reset_drivetrain_encoders();
    m_cycleCounter = (int) SmartDashboard.getNumber("Cycles to run", 0);
    left = SmartDashboard.getNumber("Left Speed", 0);
    right = SmartDashboard.getNumber("Right Speed", 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    m_driveTrain.teleop_drive(left, right);

    m_cycleCounter--;
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stopMotors();
    double leftEnc  = m_driveTrain.getLeftEncoderValue();
    double rightEnc = m_driveTrain.getRightEncoderValue();
    SmartDashboard.putNumber("Lenc", leftEnc);
    SmartDashboard.putNumber("Renc", rightEnc);
    double res = (rightEnc > 0) ? leftEnc/rightEnc : 0;
    SmartDashboard.putNumber("L/R",  res);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_cycleCounter <= 0);
  }

  public void getRightTalonModifier() {
    SmartDashboard.getNumber("Decrese Right Speed by", 0);
  }

  public void getLeftTalonModifier() {
    SmartDashboard.getNumber("Decrese Left Speed by", 0);
  }
}
