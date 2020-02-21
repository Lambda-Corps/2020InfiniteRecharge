/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorWheel extends SubsystemBase {
  /**
   * Creates a new ColorWheel.
   */

  private final TalonSRX m_ColorSpinner;

  private int shouldGoForward = 1;
  // private final AddressableLEDBuffer m_ledBuffer;
  // private final AddressableLED m_led;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

   private Boolean DoIHaveGameData = false;
   private Boolean IsColor = false;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // changes the t2c port for the color sensor

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  // This line tells what the sensor is going to read
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  public Color desiredColor;

  

  public ColorWheel() {
    m_ColorSpinner = new TalonSRX(COLOR_WHEEL_TALON);
    //SmartDashboard.putNumber("spinningSpeedThree", 1);
    //SmartDashboard.putNumber("spinningSpeedToColor", 0.5);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    

    // m_led = new AddressableLED(9);
    // m_ledBuffer = new AddressableLEDBuffer(60);
    // m_led.setLength(m_ledBuffer.getLength());
    // m_led.setData(m_ledBuffer);
    // m_led.start();

  }

    //This speed will spin three times
  public void startSpinningThree() {

    //final double spinningSpeed = SmartDashboard.getNumber("spinningSpeedThree", 1);
    final double spinningSpeed = SPINNING_THREE_TIMES_SPEED;
    this.m_ColorSpinner.set(ControlMode.PercentOutput, spinningSpeed * this.shouldGoForward);

  }
    //this speed will spin to a color
  public boolean SpinToColor(){
    //final double spinningSpeed = SmartDashboard.getNumber("spinningSpeedToColor", 0.5);
    final double spinningSpeed = SPINNING_TO_A_COLOR;
    this.m_ColorSpinner.set(ControlMode.PercentOutput, spinningSpeed * this.shouldGoForward);

    //read the color sensor
    final Color detectedColor = m_colorSensor.getColor();

    //if the color sensor is detecting the desired color
    final ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    GameDataIsRecived();
    if(match.color == desiredColor){
      stopSpinning();
      return true;
    }else{
      return false;
    }

  }

  public void stopSpinning() {

    this.m_ColorSpinner.set(ControlMode.PercentOutput, 0);
  }





  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final Color detectedColor = m_colorSensor.getColor();
    final ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    String ColorString;

    if (match.color == kBlueTarget){
      ColorString = "blue";

    }else if (match.color == kGreenTarget){
      ColorString = "green";
    }else if (match.color == kRedTarget
    ){
      ColorString = "red";

    }else if (match.color == kYellowTarget){
      ColorString = "yellow";
      
    }else{
      ColorString = "unknown";
    }

 
    if(this.DoIHaveGameData == true){
      GameDataIsRecived();
    }

    // SmartDashboard.putNumber("blue", detectedColor.blue);
    // SmartDashboard.putNumber("green", detectedColor.green);
    // SmartDashboard.putNumber("red", detectedColor.red);
    // SmartDashboard.putNumber("Confidence", match.confidence);
    // SmartDashboard.putString("Detected Color", ColorString);

  }
  public void GameDataIsRecived(){
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    
    
    if(gameData.length() > 0){
      this.DoIHaveGameData = true;
      switch (gameData.charAt(0)){
        case 'B':
        //Blue case code
        desiredColor = kRedTarget;
        break;
      case 'G' :
        //Green case code
        desiredColor = kYellowTarget;
        break;
      case 'R' :
        //Red case code
        desiredColor = kBlueTarget;
        break;
      case 'Y' :
        //Yellow case code
        desiredColor = kGreenTarget;
        break;
      default :
        //This is corrupt data
        break;
      }
    }else{
      //Code for not data recieved yet
    }
  }

  public Boolean isFinished(){
    return this.DoIHaveGameData;
  }

  public void setFoward(final boolean shouldGoForward) {
    if (shouldGoForward){
      this.shouldGoForward = 1;
    } else {
      this.shouldGoForward = -1;
    }
  }
}

