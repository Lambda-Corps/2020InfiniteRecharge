/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ShuffleboardInfo;

public class LEDLights extends SubsystemBase {
  private static final int i = 0;
  private final AddressableLEDBuffer m_ledDriveTrainBuffer;
  private final AddressableLED m_ledDriveTrian;

  private final AddressableLEDBuffer m_ledClimberBuffer;
  private final AddressableLED m_ledClimber;
  // public final AddressableLED m_ledShooter = new AddressableLED(9);
  // public final AddressableLED m_ledClimber = new AddressableLED(8);

  // public final AddressableLED m_ledIntake = new AddressableLED(6);
  private final NetworkTableEntry m_low_gear_entry;
  private final NetworkTableEntry m_downEntry;
  // Declare the variable from the subsystem we care about
  private boolean m_DT_isLowGear; // False means high gear, true means low gear

  // Network Table Entries

  /**
   * Creates a new LEDLights.
   */
  private boolean emergencyOff = false;
  private int m_rainbowFirstPixelHue;

  public LEDLights() {
    m_low_gear_entry = ShuffleboardInfo.getInstance().getDriverLowGearEntry();
    m_downEntry = ShuffleboardInfo.getInstance().getDownEntry();
    // m_ledShooter.setLength(m_ledBuffer.getLength());
    // m_ledClimber.setLength(m_ledBuffer.getLength());
    // m_ledIntake.setLength(m_ledBuffer.getLength());
    m_ledDriveTrian = LED_DRIVER_LOW_GEAR(7);
    m_ledDriveTrainBuffer = LED_BUFFER(20);
    m_ledClimberBuffer = LED_CLIMBER_BUFFER(20);
    m_ledClimber = LED_CLIMBER(9);

    Shuffleboard.getTab("Color").add("Subsystem", this);

  }

  private AddressableLED LED_DRIVER_LOW_GEAR(int j) {
    return m_ledDriveTrian;

  }

  private AddressableLED LED_CLIMBER(int z) {
    return m_ledClimber;
  }

  private AddressableLEDBuffer LED_BUFFER(int x) {
    return m_ledDriveTrainBuffer;

  }

  private AddressableLEDBuffer LED_CLIMBER_BUFFER(int v) {
    return m_ledClimberBuffer;

  }

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledDriveTrainBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledDriveTrainBuffer.getLength())) % 180;
      // Set the value
      m_ledDriveTrainBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  public void pink() {
    for (var i = 0; i < m_ledDriveTrainBuffer.getLength(); i++) {
      m_ledDriveTrainBuffer.setRGB(i, 225, 20, 147);
    }
  }

  public void red() {
    for (var i = 0; i < m_ledDriveTrainBuffer.getLength(); i++) {
      m_ledDriveTrainBuffer.setRGB(i, 255, 0, 0);
    }
  }

  public void orange() {
    for (var i = 0; i < m_ledDriveTrainBuffer.getLength(); i++) {
      m_ledDriveTrainBuffer.setRGB(i, 255, 165, 0);
    }
  }

  public void GreenStrip() {
    for (var i = 0; i < m_ledDriveTrainBuffer.getLength(); i++) {
      m_ledDriveTrainBuffer.setRGB(i, 0, 255, 0);
    }
  }

  public void BlueStrip() {
    for (var i = 0; i < m_ledDriveTrainBuffer.getLength(); i++) {
      m_ledDriveTrainBuffer.setRGB(i, 0, 127, 255);
    }
  }

  public void yellow() {
    for (var i = 0; i < m_ledDriveTrainBuffer.getLength(); i++) {
      m_ledDriveTrainBuffer.setRGB(i, 255, 255, 0);
    }
  }

  public void DontShowColor() {
    for (var i = 0; i < m_ledDriveTrainBuffer.getLength(); i++) {
      m_ledDriveTrainBuffer.setRGB(i, 0, 0, 0);
    }
  }

  @Override
  public void periodic() {

  }

  public void robotInit() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    if (m_low_gear_entry.getBoolean(false)) {
      orange();
    } else {
      GreenStrip();
    }

    if (m_downEntry.getBoolean(false)) {
      orange();
    } else {
      GreenStrip();
    }
    // Set the data
    m_ledDriveTrian.setData(m_ledDriveTrainBuffer);
    m_ledDriveTrian.start();
    m_ledClimber.setData(m_ledClimberBuffer);
    m_ledClimber.start();
    m_ledDriveTrian.setLength(m_ledDriveTrainBuffer.getLength());
    m_ledClimber.setLength(m_ledClimberBuffer.getLength());
  }

  // private void setShootingLED(){
  // if(m_isShootingEntry){
  // yellow();
  // }
  // else{
  // red();
  // }
  // }

}
