/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.BOTTEM_LIMIT_SWITCH;
import static frc.robot.Constants.CLIMBER_CHANNEL_A;
import static frc.robot.Constants.CLIMBER_CHANNEL_B;
import static frc.robot.Constants.CLIMBER_MOTOR;
import static frc.robot.Constants.TOP_LIMIT_SWITCH;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  // Settings
  private final double RAISE_SPEED = 1.0;
  private final double LOWER_SPEED = -1.0;

  // Components of the climber.
  private final DoubleSolenoid m_solenoid;
  private final DigitalInput m_toplimitswitch;
  private final DigitalInput m_bottomlimitswitch;
  private final TalonSRX m_motor;

  // Working variables.
  //private boolean emergencyOff = false;

  /**
   * Represents the climber of Team 1895's robot. It handles the raising,
   * lowering, and locking of the elevator to allow climbing on the power switch
   * at the end of a match.
   */
  public Climber() {
    this.m_toplimitswitch = new DigitalInput(TOP_LIMIT_SWITCH);
    this.m_bottomlimitswitch = new DigitalInput(BOTTEM_LIMIT_SWITCH);
    this.m_motor = new TalonSRX(CLIMBER_MOTOR);
    this.m_solenoid = new DoubleSolenoid(CLIMBER_CHANNEL_A, CLIMBER_CHANNEL_B);

    this.m_motor.setInverted(false); // Positive Voltage = Climber Raise

    Shuffleboard.getTab("Climber").add("top switch", m_toplimitswitch);
    Shuffleboard.getTab("Climber").add("bottom switch", m_bottomlimitswitch);
    Shuffleboard.getTab("Climber").add("Subsystem", this);
    Shuffleboard.getTab("Climber").add("Solenoid", m_solenoid);
  }

  /**
   * Checks if the climber is at its lowest position.
   * 
   * @return A boolean indicating if the climber is at its lowest position.
   */
  public boolean isLowerLimitReached() {
    return m_bottomlimitswitch.get();
  }

  /**
   * Checks if the climber is at its highest position.
   * 
   * @return A boolean indicating if the climber is at its highest position.
   */
  public boolean isUpperLimitReached() {
    return m_toplimitswitch.get();
  }

  /**
   * Commands the climber to start raising. Will refuse and stop the motor if the
   * climber is at its upper limit.
   */
  public void raise() {
    if (!this.isUpperLimitReached()) {
      this.setMotorSpeed(RAISE_SPEED);
    } else {
      this.setMotorSpeed(0);
    }
  }

  /**
   * Commands the climber to start lowering. Will refuse and stop the motor if the
   * climber is at its lower limit.
   */
  public void lower() {
    if (!this.isLowerLimitReached()) {
      this.setMotorSpeed(LOWER_SPEED);
    } else {
      this.setMotorSpeed(0);
    }
  }

  /**
   * Internal method for setting the speed of the motor. Will ensure the locking
   * pistons are retracted as they will destroy the mechanism if they are extended
   * while the motor is powered.
   * 
   * @param speed A number between -1.0 and 1.0 indicating the speed. A negative
   *              number will lower the climber, and positive number will raise
   *              the climber, and 0 will stop the motor.
   */
  private void setMotorSpeed(double speed) {
    this.retractLockingPistons(); // Never allow the locking pistons to be extended while motor is moving.

    this.m_motor.set(ControlMode.PercentOutput, speed);
  }

  public void stopMotor() {
    this.m_motor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Commands the locking pistons to extend. Will stop the motor as extending the
   * locking pistons while the motor is running will destroy the climber.
   */
  public void extendLockingPistons() {
    this.stopMotor(); // Never allow the motor to be running while locking pistons are extended.

    this.m_solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Commands the locking pistons to retract, allowing the climber to move freely.
   */
  public void retractLockingPistons() {
    this.m_solenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void periodic() {
  }
}
