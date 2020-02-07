/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class ClimberDown extends CommandBase {
  final Climber m_climber;
  final XboxController m_conntroler;
  private double m_speed;
  private boolean m_isdonedown;
  //private boolean solenoid;
  
  public ClimberDown (Climber climber ,  XboxController driverController) {
    m_climber = climber;
    m_conntroler = driverController;
        // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_speed = m_climber.getdownspeed();

    
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //boolean m_Solenoid = false;
    m_climber.drivedown(m_speed);
    m_isdonedown = m_climber.isclimberdonedown();
    
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopmotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isdonedown;
  }
}

