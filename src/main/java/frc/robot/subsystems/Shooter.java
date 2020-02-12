/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
   private final TalonSRX Shooter;
  // private final Encoder ShooterEncoder;
  public Shooter() {
    Shooter = new TalonSRX(2);
    Shooter.setSensorPhase(false);
    Shooter.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void velocityPID(double setpoint, double tolerance) {
    Shooter.set(ControlMode.Velocity, setpoint);
  }

  public void stopMotors() {
    Shooter.set(ControlMode.PercentOutput, 0);
  }

  public void configureVelocityPID(double kp, double ki, double kd, double kf) {
    Shooter.config_kP(0, kp);
    Shooter.config_kI(0, ki);
    Shooter.config_kD(0, kd);
    Shooter.config_kF(0, kf);
  }

}
