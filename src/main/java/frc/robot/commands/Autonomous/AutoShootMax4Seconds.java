/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotDistance;
import static frc.robot.Constants.*;

public class AutoShootMax4Seconds extends CommandBase {
  /**
   * Creates a new ShooterCommand.
   */
  private Shooter m_shooter;
  private Intake m_intake;
  private Timer cmdTimer;
  private ShotDistance m_distance;

  private int m_conveyor_empty_count;

  
  public AutoShootMax4Seconds(Shooter shooter, Intake intake, ShotDistance distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_intake = intake;
    m_distance = distance;
    addRequirements(m_shooter, m_intake);
    cmdTimer = new Timer();
    m_conveyor_empty_count = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdTimer.reset();
    cmdTimer.start();
    m_shooter.setShotDistance(m_distance);
    m_conveyor_empty_count = 0;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Start the shooter 
    m_shooter.shootFromDistance(m_distance);
    if( cmdTimer.get() >= SHOOTER_RAMP_TIME){
      // Turn on the intake motors after the delay has been met.
      m_intake.shootBalls();
    }
    
    if( m_intake.conveyorColumnEmpty() ){
      m_conveyor_empty_count++;
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
    return ( (m_conveyor_empty_count >= 5) || cmdTimer.get() > 4);
  }
}
