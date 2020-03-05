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
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.ShuffleboardInfo;
public class DriveTrain extends SubsystemBase {
  private double m_quickStopThreshold = .2;
  private double m_quickStopAlpha = .1;
  private double m_quickStopAccumulator;
  private double m_deadband = .1;

  private final WPI_TalonSRX m_rightLeader, m_rightFollower;
  private final WPI_TalonSRX m_leftLeader, m_leftFollower;
  private final TalonSRXConfiguration m_rightConfig, m_leftConfig;
  private final DoubleSolenoid m_gearbox;
  private final DifferentialDrive m_safety_drive;

  private static final int kPIDLoopIdx = 0;


  private final NetworkTableEntry m_low_gear_entry;
  private double m_last_heading;
  private final AHRS navx;
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    // Get shuffleboard components
    m_low_gear_entry = ShuffleboardInfo.getInstance().getDriverLowGearEntry();
    m_rightLeader = new WPI_TalonSRX(RIGHT_TALON_LEADER);
    m_rightFollower = new WPI_TalonSRX(RIGHT_TALON_FOLLOWER);
    m_rightConfig = new TalonSRXConfiguration();
    m_rightLeader.configAllSettings(m_rightConfig);
    m_rightFollower.configAllSettings(m_rightConfig);

    m_rightConfig.openloopRamp = DT_OPENLOOP_RAMP_RATE;
    m_rightConfig.continuousCurrentLimit = DT_CONTINUOUS_CURRENT;
    m_rightConfig.nominalOutputForward = 0;
    m_rightConfig.nominalOutputReverse = 0;
    m_rightConfig.peakOutputForward = 1;
    m_rightConfig.peakOutputReverse = -1;
    m_rightConfig.slot0.kF = kGains_DriveMM.kF;
    m_rightConfig.slot0.kP = kGains_DriveMM.kP;
    m_rightConfig.slot0.kI = kGains_DriveMM.kI;
    m_rightConfig.slot0.kD = kGains_DriveMM.kD;
    m_rightConfig.slot0.allowableClosedloopError = 10;
    m_rightConfig.slot1.kF = kGains_TurnMM_big.kF;
    m_rightConfig.slot1.kP = kGains_TurnMM_big.kP;
    m_rightConfig.slot1.kI = kGains_TurnMM_big.kI;
    m_rightConfig.slot1.kD = kGains_TurnMM_big.kD;
    m_rightConfig.slot1.allowableClosedloopError = 10;
    m_rightConfig.motionCruiseVelocity = DT_MOTION_CRUISE_VEL;
    m_rightConfig.motionAcceleration = DT_MOTION_ACCELERATION;
    m_rightLeader.configAllSettings(m_rightConfig);
    
    m_rightLeader.setNeutralMode(NeutralMode.Brake);
    m_rightLeader.setSensorPhase(true);
    m_rightLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_rightLeader.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
    m_rightFollower.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.follow(m_rightLeader);

    // Phoenix Tuner showed left side needs to be inverted
    m_leftLeader  = new WPI_TalonSRX(LEFT_TALON_LEADER);
    m_leftFollower = new WPI_TalonSRX(LEFT_TALON_FOLLOWER);
    m_leftConfig = new TalonSRXConfiguration();
    m_leftLeader.configAllSettings(m_leftConfig);
    m_leftFollower.configAllSettings(m_leftConfig);

    m_leftConfig.openloopRamp = DT_OPENLOOP_RAMP_RATE;
    m_leftConfig.continuousCurrentLimit = DT_CONTINUOUS_CURRENT;
    m_leftConfig.nominalOutputForward = 0;
    m_leftConfig.nominalOutputReverse = 0;
    m_leftConfig.peakOutputForward = 1;
    m_leftConfig.peakOutputReverse = -1;
    m_leftConfig.slot0.kF = kGains_DriveMM.kF;
    m_leftConfig.slot0.kP = kGains_DriveMM.kP;
    m_leftConfig.slot0.kI = kGains_DriveMM.kI;
    m_leftConfig.slot0.kD = kGains_DriveMM.kD;
    m_leftConfig.slot0.allowableClosedloopError = 10;
    m_leftConfig.slot1.kF = kGains_TurnMM_big.kF;
    m_leftConfig.slot1.kP = kGains_TurnMM_big.kP;
    m_leftConfig.slot1.kI = kGains_TurnMM_big.kI;
    m_leftConfig.slot1.kD = kGains_TurnMM_big.kD;
    m_leftConfig.slot1.allowableClosedloopError = 10;
    m_leftConfig.motionCruiseVelocity = DT_MOTION_CRUISE_VEL;
    m_leftConfig.motionAcceleration = DT_MOTION_ACCELERATION;
    m_leftLeader.configAllSettings(m_leftConfig);

    m_leftLeader.setNeutralMode(NeutralMode.Brake);
    m_leftLeader.setSensorPhase(true);
    m_leftLeader.setInverted(true);
    m_leftLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_leftLeader.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
    m_leftFollower.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setInverted(InvertType.FollowMaster);
    m_leftFollower.follow(m_leftLeader);

		// select profile slot
		m_leftLeader.selectProfileSlot(DT_SLOT_DRIVE_MM, kPIDLoopIdx);
    m_rightLeader.selectProfileSlot(DT_SLOT_DRIVE_MM, kPIDLoopIdx);
    
	  //m_leftLeader.config_kF(DT_SLOT_DRIVE_MM, 0.1778511821974965229, kTimeoutMs);
		//m_rightLeader.config_kF(DT_SLOT_DRIVE_MM, 0.18686069167072576716999, kTimeoutMs);

    // We have observed a couple times where the robot loses control and continues without operator
    // input, changed the TalonSRX objects to be WPI_Talons so we can use the differential drive.
    // We aren't going to actually drive with it.  We are just going to use it for the Watchdog timer.
    m_safety_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);

    m_gearbox = new DoubleSolenoid(GEARBOX_SOLENOID_A, GEARBOX_SOLENOID_B);

    navx = new AHRS(SPI.Port.kMXP);
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
    // SmartDashboard.putNumber("Left Velocity", m_leftLeader.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Right Velocity", m_rightLeader.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Left Encoder", m_leftLeader.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Right Encoder", m_rightLeader.getSelectedSensorPosition());
    // This method will be called once per scheduler run
    m_low_gear_entry.forceSetBoolean(isLowGear());

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

    m_leftLeader.configMotionCruiseVelocity(2100, kTimeoutMs); 
    m_leftLeader.configMotionAcceleration(500, kTimeoutMs); // cruise velocity / 2, so it will take 2 seconds
		m_rightLeader.configMotionCruiseVelocity(2100, kTimeoutMs); 
    m_rightLeader.configMotionAcceleration(500, kTimeoutMs); // cruise velocity / 2, so it will take 2 seconds
    
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
    m_leftLeader.selectProfileSlot(DT_SLOT_TURN_MM, PID_PRIMARY);
    m_rightLeader.selectProfileSlot(DT_SLOT_TURN_MM, PID_PRIMARY);
    m_leftLeader.configMotionCruiseVelocity(3000, kTimeoutMs);
    m_leftLeader.configMotionAcceleration(3000, kTimeoutMs);
    m_rightLeader.configMotionCruiseVelocity(3000, kTimeoutMs);
    m_rightLeader.configMotionAcceleration(3000, kTimeoutMs);
  }

  public void motion_magic_end_config_turn(){
    m_leftLeader.configMotionCruiseVelocity(2100, kTimeoutMs);
    m_leftLeader.configMotionAcceleration(500, kTimeoutMs);
    m_rightLeader.configMotionCruiseVelocity(2100, kTimeoutMs);
    m_rightLeader.configMotionAcceleration(500, kTimeoutMs);
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

    int currentL = Math.abs(m_leftLeader.getSelectedSensorPosition());
    int currentR = Math.abs(m_rightLeader.getSelectedSensorPosition());
    int targetTicks = Math.abs(arc_in_ticks);

    return (targetTicks - currentL) < tolerance && (targetTicks - currentR) < tolerance;
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

   //shifting
   public void setLowGear(){
    m_gearbox.set(DT_LOW_GEAR);
  }

  public void setHighGear(){
    m_gearbox.set(DT_HIGH_GEAR);
  }

  public void autoShiftGears(){

    int leftSpeed = m_leftLeader.getSelectedSensorVelocity();
    int rightSpeed = m_rightLeader.getSelectedSensorVelocity();

    leftSpeed = Math.abs(leftSpeed);
    rightSpeed = Math.abs(rightSpeed);

    if( leftSpeed > UP_SHIFT_SPEED && rightSpeed > UP_SHIFT_SPEED ){
      // Should be in High Gear 
      setHighGear();
    }
    else if ( leftSpeed < DOWN_SHIFT_SPEED || rightSpeed < DOWN_SHIFT_SPEED ){
      // Any other condition we should probably be in low gear, this includes when
      // one side is turning faster than the other.
      setLowGear();
    }
  }

  public boolean isLowGear(){
    return (m_gearbox.get() == DT_LOW_GEAR);
  }

  public void disableMotorSafety(){
    m_safety_drive.setSafetyEnabled(false);
  }

  public void enableMotorSafety(){
    m_safety_drive.setSafetyEnabled(true);
  }
  public void feedWatchdog(){
    m_safety_drive.feed();
  }

  public double get_last_heading() {
    return m_last_heading;
  }

  public void reset_gyro(){
    navx.reset();
  }

  public void set_last_heading(){
    m_last_heading = navx.getAngle();
  }
}
