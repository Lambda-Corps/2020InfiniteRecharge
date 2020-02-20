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
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
public class DriveTrain extends SubsystemBase {
  private double m_quickStopThreshold = .2;
  private double m_quickStopAlpha = .1;
  private double m_quickStopAccumulator;
  private double m_deadband = .1;

  private final WPI_TalonSRX m_rightLeader, m_rightFollower;
  private final WPI_TalonSRX m_leftLeader, m_leftFollower;
  private final DoubleSolenoid m_gearbox;
  private final DifferentialDrive m_safety_drive;

  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 5;

  //private final AHRS navx;
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    //navx = new AHRS();
    // Finish with phoenix tuner to determine inversion, sensor phase
    // and all the rest of the Bring-Up steps for the Talon
    m_rightLeader = new WPI_TalonSRX(RIGHT_TALON_LEADER);
    m_rightLeader.configFactoryDefault();
    m_rightLeader.setNeutralMode(NeutralMode.Brake);
    m_rightLeader.setSensorPhase(true);
    m_rightFollower = new WPI_TalonSRX(RIGHT_TALON_FOLLOWER);
    m_rightFollower.configFactoryDefault();
    m_rightFollower.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.follow(m_rightLeader);

    // Phoenix Tuner showed left side needs to be inverted
    m_leftLeader  = new WPI_TalonSRX(LEFT_TALON_LEADER);
    m_leftLeader.configFactoryDefault();
    m_leftLeader.setNeutralMode(NeutralMode.Brake);
    m_leftLeader.setSensorPhase(true);
    m_leftLeader.setInverted(true);
    m_leftFollower = new WPI_TalonSRX(LEFT_TALON_FOLLOWER);
    m_leftFollower.configFactoryDefault();
    m_leftFollower.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setInverted(InvertType.FollowMaster);
    m_leftFollower.follow(m_leftLeader);

    // Setup the power metering for the robot
    m_leftLeader.configOpenloopRamp(DT_OPENLOOP_RAMP_RATE);
    m_leftFollower.configOpenloopRamp(DT_OPENLOOP_RAMP_RATE);
    m_rightLeader.configOpenloopRamp(DT_OPENLOOP_RAMP_RATE);
    m_rightFollower.configOpenloopRamp(DT_OPENLOOP_RAMP_RATE);
    
    m_leftLeader.configContinuousCurrentLimit(DT_CONTINUOUS_CURRENT);
    m_rightLeader.configContinuousCurrentLimit(DT_CONTINUOUS_CURRENT);
    m_leftFollower.configContinuousCurrentLimit(DT_CONTINUOUS_CURRENT);
    m_rightFollower.configContinuousCurrentLimit(DT_CONTINUOUS_CURRENT);

    m_gearbox = new DoubleSolenoid(GEARBOX_SOLENOID_A, GEARBOX_SOLENOID_B);
    setLowGear();

    // Talon Speed Modifiers

    		// Set up Motion Magic
		// nominal output forward (0)
		m_leftLeader.configNominalOutputForward(0, kTimeoutMs);
		m_rightLeader.configNominalOutputForward(0, kTimeoutMs);
		// nominal output reverse (0)
		m_leftLeader.configNominalOutputReverse(0, kTimeoutMs);
		m_rightLeader.configNominalOutputReverse(0, kTimeoutMs);
		// peak output forward (1)
		m_leftLeader.configPeakOutputForward(OPEN_LOOP_PEAK_OUTPUT_F, kTimeoutMs);
		m_rightLeader.configPeakOutputForward(OPEN_LOOP_PEAK_OUTPUT_F, kTimeoutMs);
		// peak output reverse (-1)
		m_leftLeader.configPeakOutputReverse(OPEN_LOOP_PEAK_OUTPUT_B, kTimeoutMs);
		m_rightLeader.configPeakOutputReverse(OPEN_LOOP_PEAK_OUTPUT_B, kTimeoutMs);
		// select profile slot
		m_leftLeader.selectProfileSlot(DT_SLOT_DRIVE_MM, kPIDLoopIdx);
    m_rightLeader.selectProfileSlot(DT_SLOT_DRIVE_MM, kPIDLoopIdx);
    
    // Setup MotionMagic pid values for Drive and then Turn
		// config DriveMM pidf values
		m_leftLeader.config_kF(DT_SLOT_DRIVE_MM, kGains_DriveMM.kF, kTimeoutMs);
		m_leftLeader.config_kP(DT_SLOT_DRIVE_MM, kGains_DriveMM.kP, kTimeoutMs);
		m_leftLeader.config_kI(DT_SLOT_DRIVE_MM, kGains_DriveMM.kI, kTimeoutMs);
		m_leftLeader.config_kD(DT_SLOT_DRIVE_MM, kGains_DriveMM.kD, kTimeoutMs);
		m_rightLeader.config_kF(DT_SLOT_DRIVE_MM, kGains_DriveMM.kF, kTimeoutMs);
		m_rightLeader.config_kP(DT_SLOT_DRIVE_MM, kGains_DriveMM.kP, kTimeoutMs);
		m_rightLeader.config_kI(DT_SLOT_DRIVE_MM, kGains_DriveMM.kI, kTimeoutMs);
    m_rightLeader.config_kD(DT_SLOT_DRIVE_MM, kGains_DriveMM.kD, kTimeoutMs);
    
    // config TurnMM pidf values, default to turns > 45 degrees
		m_leftLeader.config_kF(DT_SLOT_TURN_MM, kGains_TurnMM_big.kF, kTimeoutMs);
		m_leftLeader.config_kP(DT_SLOT_TURN_MM, kGains_TurnMM_big.kP, kTimeoutMs);
		m_leftLeader.config_kI(DT_SLOT_TURN_MM, kGains_TurnMM_big.kI, kTimeoutMs);
		m_leftLeader.config_kD(DT_SLOT_TURN_MM, kGains_TurnMM_big.kD, kTimeoutMs);
		m_rightLeader.config_kF(DT_SLOT_TURN_MM, kGains_TurnMM_big.kF, kTimeoutMs);
		m_rightLeader.config_kP(DT_SLOT_TURN_MM, kGains_TurnMM_big.kP, kTimeoutMs);
		m_rightLeader.config_kI(DT_SLOT_TURN_MM, kGains_TurnMM_big.kI, kTimeoutMs);
    m_rightLeader.config_kD(DT_SLOT_TURN_MM, kGains_TurnMM_big.kD, kTimeoutMs);
    
		// config cruise velocity, acceleration
    m_leftLeader.configMotionCruiseVelocity(3000, kTimeoutMs); 
		m_leftLeader.configMotionAcceleration(1500, kTimeoutMs); // cruise velocity / 2, so it will take 2 seconds
		m_rightLeader.configMotionCruiseVelocity(3000, kTimeoutMs); 
		m_rightLeader.configMotionAcceleration(1500, kTimeoutMs); // cruise velocity / 2, so it will take 2 seconds
		

		m_leftLeader.configAllowableClosedloopError(0, 10, 3);
		m_rightLeader.configAllowableClosedloopError(0, 10, 3);

		// Set the quadrature encoders to be the source feedback device for the talons
		m_leftLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		m_rightLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		// reset sensors
		m_leftLeader.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
    m_rightLeader.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

    // We have observed a couple times where the robot loses control and continues without operator
    // input, changed the TalonSRX objects to be WPI_Talons so we can use the differential drive.
    // We aren't going to actually drive with it.  We are just going to use it for the Watchdog timer.
    m_safety_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);
  }

  private double normalize(double speed) {
    if (speed > 1) {
      speed = 1;
    } else if (speed < -1) {
      speed = -1;
    }
    if (speed >= CONTROLLER_DEADBAND_NEGATIVE && speed <= CONTROLLER_DEADBAND_POSOTIVE) {
      speed = 0;
    }
    return speed;
  }

  public void teleop_drive(double left, double right){
    left = normalize(left);
    right = normalize(right);
    curvature_drive_imp(left, right, (left == 0) ? true : false);
    autoShiftGears();
    m_safety_drive.feed();
  }

  /* private void tank_drive_imp(double left_speed, double right_speed){
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
    
    if (leftMotorOutput < 0) {
      leftMotorOutput *= DT_REVERSE_L_MODIFIER;
    } else {
      leftMotorOutput *= DT_FORWARD_L_MODIFIER;
    }

    m_leftLeader.set(ControlMode.PercentOutput, leftMotorOutput);
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
    SmartDashboard.putNumber("Left Velocity", m_leftLeader.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Velocity", m_rightLeader.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Left Encoder", m_leftLeader.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Encoder", m_rightLeader.getSelectedSensorPosition());
    // This method will be called once per scheduler run
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
  public void reset_drivetrain_encoders() {
    m_leftLeader.setSelectedSensorPosition(0, 0, 0);
    m_rightLeader.setSelectedSensorPosition(0, 0, 0);
  }

  //Motion Magic
  public void motion_magic_start_config_drive(boolean isForward){
    reset_drivetrain_encoders();
    // Setup the Talon to use Drive MM slots
    m_leftLeader.selectProfileSlot(DT_SLOT_DRIVE_MM, PID_PRIMARY);
    m_rightLeader.selectProfileSlot(DT_SLOT_DRIVE_MM, PID_PRIMARY);

    if( isForward ){
      m_leftLeader.config_kF(DT_SLOT_DRIVE_MM, kGains_DriveMM.kF);
      m_rightLeader.config_kF(DT_SLOT_DRIVE_MM, kGains_DriveMM.kF);
    }else {
      m_leftLeader.config_kF(DT_SLOT_DRIVE_MM, kGains_DriveMM.kF * -1);
      m_rightLeader.config_kF(DT_SLOT_DRIVE_MM, kGains_DriveMM.kF * -1);
    }
  }
  public void motion_magic_end_config_drive(){
    
  }

  public void motion_magic_start_config_turn(double degrees){
    if( degrees > 45 ){
      // config big
    }
    else {
      // config small
    }
    m_leftLeader.selectProfileSlot(DT_SLOT_TURN_MM, PID_PRIMARY);
    m_rightLeader.selectProfileSlot(DT_SLOT_TURN_MM, PID_PRIMARY);
  }

  public void motion_magic_end_config_turn(){

  }
  
  public int getLeftEncoderValue(){
    return m_leftLeader.getSelectedSensorPosition();
  }

  public int getRightEncoderValue(){
    return m_rightLeader.getSelectedSensorPosition();
  }

  public void reset_drive_PID_values(double kP, double kI, double kD) {
    m_leftLeader.config_kP(DT_SLOT_DRIVE_MM, kP);
    m_leftLeader.config_kI(DT_SLOT_DRIVE_MM, kI);
    m_leftLeader.config_kD(DT_SLOT_DRIVE_MM, kD);
    
    m_rightLeader.config_kP(DT_SLOT_DRIVE_MM, kP);
    m_rightLeader.config_kI(DT_SLOT_DRIVE_MM, kI);
    m_rightLeader.config_kD(DT_SLOT_DRIVE_MM, kD);
  }

  public void reset_turn_PID_values(double kP, double kI, double kD) {
    m_leftLeader.config_kP(DT_SLOT_TURN_MM, kP);
    m_leftLeader.config_kI(DT_SLOT_TURN_MM, kI);
    m_leftLeader.config_kD(DT_SLOT_TURN_MM, kD);

    m_rightLeader.config_kP(DT_SLOT_TURN_MM, kP);
    m_rightLeader.config_kI(DT_SLOT_TURN_MM, kI);
    m_rightLeader.config_kD(DT_SLOT_TURN_MM, kD);

  }

  public boolean motionMagicDrive(double target_position) {
    double tolerance = 10;
    
    m_leftLeader.set(ControlMode.MotionMagic, target_position);
		m_rightLeader.set(ControlMode.MotionMagic, target_position);

		double currentPos_L = m_leftLeader.getSelectedSensorPosition();
		double currentPos_R = m_rightLeader.getSelectedSensorPosition();

		return Math.abs(currentPos_L - target_position) < tolerance && (currentPos_R - target_position) < tolerance;
  }

  public boolean motionMagicTurn(int arc_in_ticks){
    double tolerance = 10;
    
    m_leftLeader.set(ControlMode.MotionMagic, arc_in_ticks);
		m_rightLeader.set(ControlMode.MotionMagic, -arc_in_ticks);

		double currentPos_L = m_leftLeader.getSelectedSensorPosition();
		double currentPos_R = m_rightLeader.getSelectedSensorPosition();

		return Math.abs(currentPos_L - arc_in_ticks) < tolerance && (currentPos_R + arc_in_ticks) < tolerance;
  }

  public void motion_magic_start_config_turn(int degrees){
    if( degrees > 45 ){
      // Big Turn
      // Set K,P,I,D for big turn gains
    } else {
      // Small Turn
      // Set K, P, I, D for small turns
    }

    m_leftLeader.selectProfileSlot(DT_SLOT_TURN_MM, PID_PRIMARY);
    m_rightLeader.selectProfileSlot(DT_SLOT_TURN_MM, PID_PRIMARY);
  }

  public void setLowGear(){
    m_gearbox.set(DoubleSolenoid.Value.kReverse);
  }

  public void setHighGear(){
    m_gearbox.set(DoubleSolenoid.Value.kForward);
  }

  public void autoShiftGears(){

    int leftSpeed = m_leftLeader.getSelectedSensorVelocity();
    int rightSpeed = m_rightLeader.getSelectedSensorVelocity();
    DoubleSolenoid.Value solenoidPosition = m_gearbox.get();
    double currentSpeed = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

    if((currentSpeed > UP_SHIFT_SPEED) && (solenoidPosition == Value.kForward)){
      //Low to High
      setLowGear();
    }
    else if((currentSpeed < DOWN_SHIFT_SPEED) && (solenoidPosition == Value.kReverse)){
      //High to Low
      setHighGear();
    }
  }

  @SuppressWarnings("unused")
  private boolean isLowGear(){
    return (m_gearbox.get() == DoubleSolenoid.Value.kReverse);
  }
}