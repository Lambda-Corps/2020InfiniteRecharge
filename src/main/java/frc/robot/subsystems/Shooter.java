/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  public enum ShotDistance {
    PortWall, InitiationLine, FrontTrench
  }

  /**
   * Creates a new Shooter.
   */
  private final TalonSRX m_bottomShooter, m_topShooter;
  private final TalonSRXConfiguration shooterConfig;

  private int m_shooter_set_point; // set point for shooter pid

  private final DoubleSolenoid m_hood;
  

   
  public Shooter() {
    // The shooter will shoot from different locations (optimally three),
    // so configure the Talons to have the PID gains for reach preconfigured.
    // Talon can have up to 4 PID slots, use one for each distance.
    m_bottomShooter = new TalonSRX(BOTTOM_SHOOTER);
    m_topShooter = new TalonSRX(TOP_SHOOTER);
    shooterConfig = new TalonSRXConfiguration();
    m_bottomShooter.configAllSettings(shooterConfig);
    m_topShooter.configAllSettings(shooterConfig);

    // This is the configuration for the initation line shot
    shooterConfig.slot0.kF = kGains_InitiationLine.kF;
    shooterConfig.slot0.kP = kGains_InitiationLine.kP;
    shooterConfig.slot0.kI = kGains_InitiationLine.kI;
    shooterConfig.slot0.kD = kGains_InitiationLine.kD;
    shooterConfig.slot0.integralZone = kGains_InitiationLine.kIzone;
    shooterConfig.slot0.closedLoopPeakOutput = kGains_InitiationLine.kPeakOutput;

    // Configuration to shoot from up against the wall
    shooterConfig.slot1.kF = kGains_PortWall.kF;
    shooterConfig.slot1.kP = kGains_PortWall.kP;
    shooterConfig.slot1.kI = kGains_PortWall.kI;
    shooterConfig.slot1.kD = kGains_PortWall.kD;
    shooterConfig.slot1.integralZone = kGains_PortWall.kIzone;
    shooterConfig.slot1.closedLoopPeakOutput = kGains_PortWall.kPeakOutput;

    // Configuration to shoot from the front of the trench
    shooterConfig.slot2.kF = kGains_FrontTrench.kF;
    shooterConfig.slot2.kP = kGains_FrontTrench.kP;
    shooterConfig.slot2.kI = kGains_FrontTrench.kI;
    shooterConfig.slot2.kD = kGains_FrontTrench.kD;
    shooterConfig.slot2.integralZone = kGains_FrontTrench.kIzone;
    shooterConfig.slot2.closedLoopPeakOutput = kGains_FrontTrench.kPeakOutput;
    
    // Setup the open loop limits so the drivers can't mess it up
    shooterConfig.peakOutputReverse = 0;
    shooterConfig.peakOutputForward = 1;
    shooterConfig.nominalOutputForward = 0;

    m_topShooter.configAllSettings(shooterConfig);

    m_topShooter.setInverted(false);
    m_topShooter.setSensorPhase(false);
    m_topShooter.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    // config bottom motor to follow top
    m_bottomShooter.setInverted(false);
    m_bottomShooter.follow(m_topShooter);

    // Default to the initiation line
    m_shooter_set_point = SHOOTER_SETPOINT_LINE;

    m_hood = new DoubleSolenoid(SHOOTER_CHANNEL_A, SHOOTER_CHANNEL_B);
    
    // Default to initiation line
    m_hood.set(SHOOTER_FAR_SHOT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shootFromDistance(ShotDistance distance){
    //setShotDistance(distance);
    this.m_topShooter.set(ControlMode.Velocity, m_shooter_set_point);
  }

  public void stopMotors() {
    m_topShooter.set(ControlMode.PercentOutput, 0);
  }

  // Use Slot 0 as the testing slot
  public void configureVelocityPID(double kp, double ki, double kd, double kf) {
    m_topShooter.selectProfileSlot(SHOOTER_SLOT_INITIATION_LINE, PID_PRIMARY);
    m_topShooter.config_kP(SHOOTER_SLOT_INITIATION_LINE, kp);
    m_topShooter.config_kI(SHOOTER_SLOT_INITIATION_LINE, ki);
    m_topShooter.config_kD(SHOOTER_SLOT_INITIATION_LINE, kd);
    m_topShooter.config_kF(SHOOTER_SLOT_INITIATION_LINE, kf);
  }

  // Use this for testing the shooter methods
  public void tuneVelocityPid(int setpoint){
    m_topShooter.set(ControlMode.Velocity, setpoint);
  }

  public void setShotDistance(ShotDistance distance){
    switch(distance){
      case InitiationLine:
        // Config the shooter for initiation line
        m_topShooter.selectProfileSlot(SHOOTER_SLOT_INITIATION_LINE, PID_PRIMARY);
        // Set the setpoint
        m_shooter_set_point = SHOOTER_SETPOINT_LINE;
        // Set the pistons
        m_hood.set(SHOOTER_FAR_SHOT);
        break;
      case PortWall:
        // Config the shooter for initiation line
        m_topShooter.selectProfileSlot(SHOOTER_SLOT_PORTWALL, PID_PRIMARY);
        // Set the setpoint
        m_shooter_set_point = SHOOTER_SETPOINT_WALL;
        // Set the pistons
        m_hood.set(SHOOTER_FAR_SHOT);
        break;
      case FrontTrench:
        // Config the shooter for initiation line
        m_topShooter.selectProfileSlot(SHOOTER_SLOT_FRONT_TRENCH, PID_PRIMARY);
        // Set the setpoint
        m_shooter_set_point = SHOOTER_SETPOINT_TRENCH;
        // Set the pistons
        m_hood.set(SHOOTER_FAR_SHOT);
        break;
    }
  }
}
