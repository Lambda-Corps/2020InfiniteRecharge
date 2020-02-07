/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Timer;

// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.Solenoid;

// import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


public class ClimberUp extends CommandBase {
  final Climber m_climber;
  final XboxController m_conntroler;
  private double m_speed;
  private boolean m_isdone;
  //private Solenoid m_solenoid;
  /**
   * Creates a new ClimberUp.
   */
  public ClimberUp(Climber climber, XboxController driverController) {
    m_climber = climber;
    m_conntroler = driverController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_speed = m_climber.getupspeed();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.driveup(m_speed);
    m_isdone = m_climber.isclimberdone();
    //boolean m_Solenoid = false;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_climber.stopmotor();
    
  }

  @Override
  public boolean isFinished(){
    return m_isdone;
  }
}
