/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibration;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
/*
 * This command is designed to be used for any sort of PID calibration that
 * we need to do.  It can be used with a little tweaking to be MotionMagic,
 * MotionProfile, Velocity, or Position.
 * 
 * Feel free to copy the source file to make a new one, or to extend this one
 * and make it your own. This example is using the drive train for the base 
 * subsystem, but can be used with many subsystem by changing the argument type.
 * 
 * The command will use Shuffleboard to put all of it's values that will be used
 * to tune the pid itself.  Then, on robot init, this Command can be sent to the 
 * Dashboard as well for tuning.
 */
public class ShooterTuningCommand extends CommandBase {
  private double m_setpoint, m_runTime;
  private Timer cmdTimer;
  private final Intake m_Intake;

  private final Shooter m_shooter;
  private final ShuffleboardTab m_myTab;
  private NetworkTableEntry m_kpEntry, m_kiEntry, m_kdEntry, m_kfEntry, m_spEntry, m_conveyorEntry, m_conveyorDelayEntry, m_IntakeEntry, m_IndexerEntry, m_runTimeEntry;
  /**
   * Creates a new PIDTuningCommand.
   */
  public ShooterTuningCommand(Shooter shooter, Intake intake) {
    cmdTimer = new Timer();
    m_myTab = Shuffleboard.getTab("PID Tuning");

    m_kpEntry = m_myTab.add("kP", 0).withPosition(1, 3).getEntry();
    m_kiEntry = m_myTab.add("kI", 0 ).withPosition(2, 3).getEntry();
    m_kdEntry = m_myTab.add("kD", 0 ).withPosition(3, 3).getEntry();
    m_kfEntry = m_myTab.add("kF", 0 ).withPosition(0, 3).getEntry();

    m_spEntry = m_myTab.add("Set Point", 0 ).withPosition(4, 3).getEntry();


    m_shooter = shooter;
    m_Intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double kp = m_kpEntry.getDouble(0);
    double ki = m_kiEntry.getDouble(0);
    double kd = m_kdEntry.getDouble(0);
    double kf = m_kfEntry.getDouble(0);
    m_setpoint = m_spEntry.getDouble(0);
    
    cmdTimer.reset();
    cmdTimer.start();
  

    m_shooter.configureVelocityPID(kp, ki, kd, kf);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.tuneVelocityPid((int)m_setpoint);

    if(cmdTimer.get() >= 2){
      m_Intake.shootBalls();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Shut the motors off by driving in Percent
    m_shooter.stopMotors();
    m_Intake.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
