/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

// import java.util.Set;

// import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Climber extends SubsystemBase {
  public final DoubleSolenoid m_solenoid;
  public final DigitalInput m_TopSwitch;
  public final DigitalInput m_BottemSwitch;
  public final TalonSRX m_LifterMotor;
  private NetworkTableEntry m_downspeed;
  private NetworkTableEntry m_upspeed;
  private double m_climberspeed;

  public Climber() {
    m_TopSwitch = new DigitalInput(TOP_LIMIT_SWITCH);
    m_BottemSwitch = new DigitalInput(BOTTEM_LIMIT_SWITCH);
    m_LifterMotor = new TalonSRX(CLIMBER_MOTOR);
    m_solenoid = new DoubleSolenoid(CLIMBER_CHANNEL_A, CLIMBER_CHANNEL_B);

    m_LifterMotor.setInverted(true);
    Shuffleboard.getTab("Climber").add("top switch", m_TopSwitch);
    Shuffleboard.getTab("Climber").add("bottom switch", m_BottemSwitch);
    Shuffleboard.getTab("Climber").add("Subsystem", this);
    Shuffleboard.getTab("Climber").add("Solenoid", m_solenoid);
    m_downspeed = Shuffleboard.getTab("Climber").add("downspeed", 0.25).getEntry();
    m_upspeed = Shuffleboard.getTab("Climber").add("upspeed", 0.25).getEntry();

  }

  public boolean isclimberdonedown() {
    boolean ret = false;
    if (m_BottemSwitch.get()) {
      ret = true;

    }
    return ret;
  }

  public boolean isclimberdone() {
    boolean ret = false;

    if (m_TopSwitch.get()) {
      ret = true;
    }
    return ret;

  }

  public void driveup(double speed) {
    if (speed <= 0) {
      speed = 0;
    }
    if (speed > 1) {
      speed = 1;
    }

    if (m_TopSwitch.get()) {
      speed = 0;
    }
    m_LifterMotor.set(ControlMode.PercentOutput, speed);

  }

  public boolean drivedown() {
    return m_BottemSwitch.get();
  }
public double getupspeed(){
    return m_upspeed.getDouble(0);
  
}
public double getdownspeed(){
  return m_downspeed.getDouble(0);

}
  public void stopmotor() {
    m_LifterMotor.set(ControlMode.PercentOutput, 0);
    
    {
      m_LifterMotor.set(ControlMode.PercentOutput, m_climberspeed);
      boolean m_TopSwitch = true;
    }

  }

  public void solenoidforward() {
    m_solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void solenoidreverse() {
    m_solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drivedown(double speed) {
    if (speed < -1) {
      speed = -1;
    }
    if (speed >= 0) {
      speed = 0;
    }
    if (m_BottemSwitch.get()) {
      speed = 0;
    }
    m_LifterMotor.set(ControlMode.PercentOutput, speed);
  }

  public void toggleSolenoid() {
    Value m_value = m_solenoid.get();
    if (m_value == Value.kForward) {
      m_solenoid.set(DoubleSolenoid.Value.kReverse);
    } else {
      m_solenoid.set(DoubleSolenoid.Value.kForward);
    }
  }
}
