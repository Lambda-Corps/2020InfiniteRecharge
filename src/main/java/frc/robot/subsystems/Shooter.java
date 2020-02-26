/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private final TalonSRX m_bottomShooter, m_topShooter;
  private int shouldGoForward = 1;

   
  public Shooter() {
    m_bottomShooter = new TalonSRX(BOTTOM_SHOOTER);
    m_topShooter = new TalonSRX(TOP_SHOOTER);

    m_topShooter.configFactoryDefault();
    m_bottomShooter.configFactoryDefault();

    m_topShooter.setInverted(false);
    m_topShooter.setSensorPhase(false);
    m_topShooter.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

    // config bottom motor
    m_bottomShooter.setInverted(false);
    m_bottomShooter.follow(m_topShooter);

    // setup velocity pid configuration
    m_topShooter.config_kF(PID_PRIMARY, kGains_Shooter.kF, kTimeoutMs);
    m_topShooter.config_kP(PID_PRIMARY, kGains_Shooter.kP, kTimeoutMs);
    m_topShooter.config_kD(PID_PRIMARY, kGains_Shooter.kD, kTimeoutMs);
    m_topShooter.config_kI(PID_PRIMARY, kGains_Shooter.kI, kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void StartShooter(){
    double rotation = SETPOINT;
    this.m_topShooter.set(ControlMode.PercentOutput, rotation * this.shouldGoForward);
    
  }

  public void velocityPID(double m_setpoint, double m_tolerance) {
    m_topShooter.set(ControlMode.Velocity, m_setpoint);
    
  }

  public void stopMotors() {
    m_topShooter.set(ControlMode.PercentOutput, 0);
  }

  public void setFoward(final boolean shouldGoForward) {
    if (shouldGoForward){
      this.shouldGoForward = 1;
    } else {
      this.shouldGoForward = -1;
    }
  }

  
}
