/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.robot.Constants.*;
public class DriveTrain extends SubsystemBase {
  private double m_quickStopThreshold = .2;
  private double m_quickStopAlpha = .1;
  private double m_quickStopAccumulator;
  private double m_deadband = .1; // TODO, tune this deadband to actually work with robot

  private DoubleSolenoid m_gearbox;

  private final TalonSRX m_rightLeader, m_rightFollower;
  private final TalonSRX m_leftLeader, m_leftFollower;

  private double m_LeftTalonModifier;
  private double m_rightTalonModifier;

  //private final AHRS navx;
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    //navx = new AHRS();
    // Finish with phoenix tuner to determine inversion, sensor phase
    // and all the rest of the Bring-Up steps for the Talon
    m_rightLeader = new TalonSRX(RIGHT_TALON_LEADER);
    m_rightLeader.configFactoryDefault();
    m_rightLeader.setNeutralMode(NeutralMode.Brake);
    m_rightLeader.setSensorPhase(true);
    m_rightFollower = new TalonSRX(RIGHT_TALON_FOLLOWER);
    m_rightFollower.configFactoryDefault();
    m_rightFollower.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.follow(m_rightLeader);

    // Phoenix Tuner showed left side needs to be inverted
    m_leftLeader  = new TalonSRX(LEFT_TALON_LEADER);
    m_leftLeader.configFactoryDefault();
    m_leftLeader.setNeutralMode(NeutralMode.Brake);
    m_leftLeader.setInverted(true);
    m_leftLeader.setSensorPhase(true);
    m_leftFollower = new TalonSRX(LEFT_TALON_FOLLOWER);
    m_leftFollower.configFactoryDefault();
    m_leftFollower.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setInverted(InvertType.FollowMaster);
    m_leftFollower.follow(m_leftLeader);

    m_gearbox = new DoubleSolenoid(GEARBOX_SOLENOID_A, GEARBOX_SOLENOID_B);

    // Talon Speed Modifiers
    m_LeftTalonModifier = SmartDashboard.getNumber("Decrese Right Speed by", 0);
    m_rightTalonModifier = SmartDashboard.getNumber("Decrese Left Speed by", 0);
  }

  public void teleop_drive(double left, double right){
    curvature_drive_imp(left, right, true);
  }

  /*public void tank_drive_imp(double left_speed, double right_speed){
    m_leftLeader.set(ControlMode.PercentOutput, left_speed);
    m_rightLeader.set(ControlMode.PercentOutput, right_speed);
  }
  */
  
  private void curvature_drive_imp(double xSpeed, double zRotation, boolean isQuickTurn) {

    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    xSpeed = applyDeadband(xSpeed, m_deadband);

    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    zRotation = applyDeadband(zRotation, m_deadband);

    double angularPower;
    boolean overPower;

    if (isQuickTurn) {
      if (Math.abs(xSpeed) < m_quickStopThreshold) {
        m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
            + m_quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
      }
      overPower = true;
      angularPower = zRotation;
    } else {
      overPower = false;
      angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

      if (m_quickStopAccumulator > 1) {
        m_quickStopAccumulator -= 1;
      } else if (m_quickStopAccumulator < -1) {
        m_quickStopAccumulator += 1;
      } else {
        m_quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = xSpeed + angularPower;
    double rightMotorOutput = xSpeed - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }

    m_leftLeader.set(ControlMode.PercentOutput, leftMotorOutput*.99);
    m_rightLeader.set(ControlMode.PercentOutput, rightMotorOutput);
  }

  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  @Override
  public void periodic() {
    
  }

  public void stopMotors() {
    m_leftLeader.set(ControlMode.PercentOutput, 0);
    m_rightLeader.set(ControlMode.PercentOutput, 0);
  }

  public boolean drivePositionPID(double setpoint, double tolerance) {
    m_leftLeader.set(ControlMode.Position, setpoint);
    m_rightLeader.set(ControlMode.Position, setpoint);

    int leftErr = m_leftLeader.getClosedLoopError();
    int rightErr = m_rightLeader.getClosedLoopError();

    return (leftErr <= tolerance) && (rightErr <= tolerance);
  }

  public void setEncodersToZero(){  // Set the left and right encoders to 0
    m_leftLeader.setSelectedSensorPosition(0);
    m_rightLeader.setSelectedSensorPosition(0);
  }

  public double getLeftEncoderValue(){  // Get the value of the left encoder
    return m_leftLeader.getSelectedSensorPosition();
  }

  public double getRightEncoderVlaue(){  // Get the right value of the right encoder
    return m_rightLeader.getSelectedSensorPosition();
  }

  public double getRightTalonSpeed(){
    return m_rightLeader.getSelectedSensorVelocity();
  }

  public double getLeftTalonSpeed(){
    return m_leftLeader.getSelectedSensorVelocity();
  }
}
