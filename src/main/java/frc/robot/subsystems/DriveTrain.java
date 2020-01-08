/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
public class DriveTrain extends SubsystemBase {

  private final TalonSRX m_rightLeader, m_rightFollower;
  private final TalonSRX m_leftLeader, m_leftFollower;
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    // TODO -- Finish with phoenix tuner to determine inversion, sensor phase
    // and all the rest of the Bring-Up steps for the Talon
    m_rightLeader = new TalonSRX(RIGHT_TALON_LEADER);
    m_rightFollower = new TalonSRX(RIGHT_TALON_FOLLOWER);
    m_rightFollower.follow(m_rightLeader);

    m_leftLeader  = new TalonSRX(LEFT_TALON_LEADER);
    m_leftFollower = new TalonSRX(LEFT_TALON_FOLLOWER);
    m_leftFollower.follow(m_leftLeader);
  }

  public void teleop_drive(double left, double right){
    tank_drive_imp(left, right);
  }

  private void tank_drive_imp(double left_speed, double right_speed){
    m_leftLeader.set(ControlMode.PercentOutput, left_speed);
    m_rightLeader.set(ControlMode.PercentOutput, right_speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
