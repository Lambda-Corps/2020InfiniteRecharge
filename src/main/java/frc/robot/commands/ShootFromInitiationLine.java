/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotDistance;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Timer;


public class ShootFromInitiationLine extends CommandBase {
  /**
   * Creates a new ShooterCommand.
   */
    private Shooter m_shooter;
    private Intake m_intake;
    private Timer cmdTimer;

    
  public ShootFromInitiationLine(Shooter shooter, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_intake = intake;
    addRequirements(m_shooter, m_intake);
    cmdTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdTimer.reset();
    cmdTimer.start();

    m_shooter.setShotDistance(ShotDistance.InitiationLine);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Start the shooter 
    m_shooter.shootFromDistance(ShotDistance.InitiationLine);
    if( cmdTimer.get() >= SHOOTER_RAMP_TIME){
      // Turn on the intake motors after the delay has been met.
      m_intake.shootBalls();
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopMotors();
  }

  // Command will run until the button is released, or autonomous timer hits
  @Override
  public boolean isFinished() {
    return false;
  }
}
