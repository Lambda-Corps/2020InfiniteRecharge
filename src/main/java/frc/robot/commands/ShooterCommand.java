/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Timer;


public class ShooterCommand extends CommandBase {
  /**
   * Creates a new ShooterCommand.
   */
    private Shooter m_shooter;
    private Intake m_iIntake;
    private Timer cmdTimer;
    private double m_setpoint, m_tolerance;
    private double m_runTime = RUNTIME;
    private double m_Intake, m_conveyorSpeed, m_Indexer;
    
  public ShooterCommand(Shooter subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = new Shooter();
    m_iIntake = new Intake();
    addRequirements(m_shooter);
    cmdTimer = new Timer();
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    m_runTime = RUNTIME;
    m_setpoint = SETPOINT;
    m_Intake = INTAKE_SPEED;
    m_Indexer = INDEXER_SPEED;
    m_conveyorSpeed = CONVEYOR_SPEED;
    cmdTimer.reset();
    cmdTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.velocityPID(m_setpoint, m_tolerance);
    m_shooter.StartShooter();
    if( cmdTimer.get() >= m_conveyorDelay){
      // Turn on the conveyor motor after the delay has been meet.
      m_iIntake.conveyorMotorspeed(m_conveyorSpeed);
      m_iIntake.IntakeSpeed(m_Intake);
      m_iIntake.indexerSpeed(m_Indexer);
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmdTimer.get() > m_runTime;
  }
}
