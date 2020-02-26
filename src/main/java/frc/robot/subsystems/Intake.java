/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//make deploy intake, retract intake= put on smartdashboard

public class Intake extends SubsystemBase {
  public enum DeployState {
		DEPLOY, STOW, DEPLOYING
	}
	
  private DoubleSolenoid intakePistons;
  private TalonSRX intakeMotor, conveyorMotor, indexer;
  //private DeployState deployState = DeployState.STOW;
  private DigitalInput m_TopBeam, m_MiddleTopBeam, m_MiddleBottomBeam, m_BottomBeam, m_SendBeam;
  private int shouldGoForward = 1;
  private boolean m_intakeup;

  /**
   * Creates a new Intake.
   */
  public Intake(){
    // Initialize Member Variables
    intakePistons = new DoubleSolenoid(INTAKE_SOLENOID_A, INTAKE_SOLENOID_B);
    intakeMotor = new TalonSRX(INTAKE); 
    intakeMotor.configFactoryDefault();
    conveyorMotor = new TalonSRX(INTAKE_CONVEYOR);
    indexer = new TalonSRX(INTAKE_INDEXER); 
    m_SendBeam = new DigitalInput(BEAM_BREAKER_SEND);
    m_TopBeam = new DigitalInput(BEAM_BREAKER_RECEIVER_TOP);
    m_MiddleTopBeam = new DigitalInput(BEAM_BREAKER_RECEIVER_MIDDLETOP);
    m_MiddleBottomBeam = new DigitalInput(BEAM_BREAKER_RECEIVER_MIDDLEBOTTOM);
    m_BottomBeam = new DigitalInput(BEAM_BREAKER_RECEIVER_BOTTOM);

    Shuffleboard.getTab("Intake").add("Solenoid", intakePistons);
    Shuffleboard.getTab("Intake").add("Top", m_TopBeam);
    Shuffleboard.getTab("Intake").add("Middle Top", m_MiddleTopBeam);
    Shuffleboard.getTab("Intake").add("Middle Bottom", m_MiddleBottomBeam);
    Shuffleboard.getTab("Intake").add("Bottom Beam", m_BottomBeam);
    Shuffleboard.getTab("Intake").add("Send", m_SendBeam);
    Shuffleboard.getTab("Intake").add("Intake UP", m_intakeup);
 

  }
  

  // Public method for commands to start the intake motors to collect balls
  public void conveyorMotorspeed(){
    double ConveyorSpeed = CONVEYOR_SPEED;
    this.conveyorMotor.set(ControlMode.PercentOutput, ConveyorSpeed * this.shouldGoForward);
  }

  public void indexerSpeed(){
    double IndexerSpeed = INDEXER_SPEED;
    this.indexer.set(ControlMode.PercentOutput, IndexerSpeed * this.shouldGoForward);
  }

  public void IntakeSpeed(){
    double intakeSpeed = INTAKE_SPEED;
    this.indexer.set(ControlMode.PercentOutput, intakeSpeed * this.shouldGoForward);
  }
  
  // Public method for commands to start the motors in reverse to eject balls
  public void EjectBalls(){
    intakeMotor.set(ControlMode.PercentOutput, -.5);
  }
  
  public void IntakeUP(){
    intakePistons.set(INTAKE_UP_POSITION);
  }
  public void IntakeDown() {
    intakePistons.set(INTAKE_DOWN_POSITION);
  }  


  public void setFoward(final boolean shouldGoForward) {
    if (shouldGoForward){
      this.shouldGoForward = 1;
    } else {
      this.shouldGoForward = -1;
    }
  }


public void shootBalls() {
}

}




