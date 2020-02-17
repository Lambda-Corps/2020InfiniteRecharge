/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

public class DriveTrain extends SubsystemBase {
  private double m_quickStopThreshold = .2;
  private double m_quickStopAlpha = .1;
  private double m_quickStopAccumulator;
  private double m_deadband = .1;

  private final TalonSRX m_rightLeader, m_rightFollower;
  private final TalonSRX m_leftLeader, m_leftFollower;
  private final DoubleSolenoid m_gearbox;

  private static final int kTimeoutMs = 5;

  private boolean m_isPIDCorrecting;
  private int m_auxPidTarget;

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
    m_rightFollower = new TalonSRX(RIGHT_TALON_FOLLOWER);
    m_rightFollower.configFactoryDefault();
    m_rightFollower.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.follow(m_rightLeader);

    // Phoenix Tuner showed left side needs to be inverted
    m_leftLeader  = new TalonSRX(LEFT_TALON_LEADER);
    m_leftLeader.configFactoryDefault();
    m_leftLeader.setNeutralMode(NeutralMode.Brake);
    m_leftLeader.setInverted(true);
    m_leftFollower = new TalonSRX(LEFT_TALON_FOLLOWER);
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
		// Set the minimums for forward and back
		m_leftLeader.configNominalOutputForward(0, kTimeoutMs);
		m_rightLeader.configNominalOutputForward(0, kTimeoutMs);
		m_leftLeader.configNominalOutputReverse(0, kTimeoutMs);
		m_rightLeader.configNominalOutputReverse(0, kTimeoutMs);
		// Setup the forward and reverse peak outputs
		m_leftLeader.configPeakOutputForward(DT_OPEN_LOOP_PEAK_OUTPUT_F, kTimeoutMs);
		m_rightLeader.configPeakOutputForward(DT_OPEN_LOOP_PEAK_OUTPUT_F, kTimeoutMs);
		m_leftLeader.configPeakOutputReverse(DT_OPEN_LOOP_PEAK_OUTPUT_B, kTimeoutMs);
    m_rightLeader.configPeakOutputReverse(DT_OPEN_LOOP_PEAK_OUTPUT_B, kTimeoutMs);
    
		// Set the profile slot to PID Correcting as the initial
		m_leftLeader.selectProfileSlot(DT_SLOT_AUXILIARY_PID, PID_AUXILIARY);
		m_rightLeader.selectProfileSlot(DT_SLOT_AUXILIARY_PID, PID_AUXILIARY);
		// config pidf values for Drive MM
		m_leftLeader.config_kF(DT_SLOT_AUXILIARY_PID,  kGains_AuxPID.kF, kTimeoutMs);
		m_leftLeader.config_kP(DT_SLOT_AUXILIARY_PID,  kGains_AuxPID.kP, kTimeoutMs);
		m_leftLeader.config_kI(DT_SLOT_AUXILIARY_PID,  kGains_AuxPID.kI, kTimeoutMs);
		m_leftLeader.config_kD(DT_SLOT_AUXILIARY_PID,  kGains_AuxPID.kD, kTimeoutMs);
		m_rightLeader.config_kF(DT_SLOT_AUXILIARY_PID, kGains_AuxPID.kF, kTimeoutMs);
		m_rightLeader.config_kP(DT_SLOT_AUXILIARY_PID, kGains_AuxPID.kP, kTimeoutMs);
		m_rightLeader.config_kI(DT_SLOT_AUXILIARY_PID, kGains_AuxPID.kI, kTimeoutMs);
    m_rightLeader.config_kD(DT_SLOT_AUXILIARY_PID, kGains_AuxPID.kD, kTimeoutMs);
    m_leftLeader.configAllowableClosedloopError(DT_SLOT_AUXILIARY_PID, 0, kTimeoutMs); // allowable error is 0, because we want to be straight
    m_rightLeader.configAllowableClosedloopError(DT_SLOT_AUXILIARY_PID, 0, kTimeoutMs); // allowable error is 0, because we want to be straight
    // reset sensors
		m_leftLeader.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
    m_rightLeader.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);

    // Setup the remote sensor for the differential auxiliary PID
    /* Configure the drivetrain's left side Feedback Sensor as a Quadrature Encoder */
		m_leftLeader.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,			// Local Feedback Source
                                                PID_PRIMARY,				// PID Slot for Source [0, 1]
                                                kTimeoutMs);				// Configuration Timeout

    /* Configure the left Talon's Selected Sensor to be a remote sensor for the right Talon */
    m_rightLeader.configRemoteFeedbackFilter(m_leftLeader.getDeviceID(),					// Device ID of Source
                                             RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
                                             REMOTE_0,							// Source number [0, 1]
                                             kTimeoutMs);						// Configuration Timeout

    /* Setup difference signal to be used for turn when performing Drive Straight with encoders */
    m_rightLeader.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, kTimeoutMs);	// Feedback Device of Remote Talon
    m_rightLeader.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, kTimeoutMs);		// Quadrature Encoder of current Talon

    /* Difference term calculated by right Talon configured to be selected sensor of turn PID */
    m_rightLeader.configSelectedFeedbackSensor(	FeedbackDevice.SensorDifference, PID_AUXILIARY, kTimeoutMs);
    
    /* configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
     * Our left side is faster in the forward position (most common) so we'll go false
     * such that the correction is adding to the right and subtracting from the left
		 */
		m_rightLeader.configAuxPIDPolarity(false, kTimeoutMs);
    
		// config cruise velocity, acceleration
    m_leftLeader.configMotionCruiseVelocity(DT_MOTIONMAGIC_CRUISE, kTimeoutMs); 
		m_leftLeader.configMotionAcceleration(DT_MOTIONMAGIC_ACCELERATION, kTimeoutMs); // cruise velocity / 2, so it will take 2 seconds
		m_rightLeader.configMotionCruiseVelocity(DT_MOTIONMAGIC_CRUISE, kTimeoutMs); 
		m_rightLeader.configMotionAcceleration(DT_MOTIONMAGIC_CRUISE, kTimeoutMs); // cruise velocity / 2, so it will take 2 seconds
		
		// Set the quadrature encoders to be the source feedback device for the talons
		m_leftLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		m_rightLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		
  }
  
  public void teleop_drive(double left, double right){
    left  = normalize(left);
    right = normalize(right);
    
    if( right != 0){
      // We are turning, either in place or while we drive so we won't correct based on encoders
      m_isPIDCorrecting = false;
      curvature_drive_imp(left, right, (left == 0) ? true : false);
    } else {
      if(! m_isPIDCorrecting ){
        // We weren't previously correcting, so treat this as the first time as we're starting.
        // Grab the current aux pid heading
        m_auxPidTarget = m_rightLeader.getSelectedSensorPosition(PID_AUXILIARY);
        // Set our profile to the auxiliary pid
        m_rightLeader.selectProfileSlot(DT_SLOT_AUXILIARY_PID, PID_AUXILIARY);
        
      }
      drive_straight_pid_corrected(left);
    }
    autoShiftGears();
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

  private void drive_straight_pid_corrected(double speed){
    /* Configured for percentOutput with Auxiliary PID on Quadrature Encoders' Difference */
			m_rightLeader.set(ControlMode.PercentOutput, speed, DemandType.AuxPID, m_auxPidTarget);
			m_leftLeader.follow(m_rightLeader, FollowerType.AuxOutput1);
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
    m_leftLeader.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
    m_rightLeader.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
  }

  //Motion Magic
  public void motion_magic_start_config_drive(){
    m_rightLeader.selectProfileSlot(DT_SLOT_DRIVE_MM, PID_PRIMARY);
    m_leftLeader.follow(m_rightLeader, FollowerType.AuxOutput1);
    m_auxPidTarget = m_rightLeader.getSelectedSensorPosition(PID_AUXILIARY);

    m_rightLeader.configPeakOutputForward(1, kTimeoutMs);
    m_rightLeader.configPeakOutputReverse(-1, kTimeoutMs);
  }
  public void motion_magic_end_config_drive(){
    // To start motion magic we set the left side to follow the right, 
    // call *.set() to get it out of follower mode.
    m_rightLeader.set(ControlMode.PercentOutput, 0);
    m_leftLeader.set(ControlMode.PercentOutput, 0);
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
    //m_leftLeader.config_kF(DT_SLOT_DRIVE_MM, kF);
    
    m_rightLeader.config_kP(DT_SLOT_DRIVE_MM, kP);
    m_rightLeader.config_kI(DT_SLOT_DRIVE_MM, kI);
    m_rightLeader.config_kD(DT_SLOT_DRIVE_MM, kD);
    //m_rightLeader.config_kF(DT_SLOT_DRIVE_MM, kF);
  }

  public void reset_turn_PID_values(double kP, double kI, double kD) {
    m_leftLeader.config_kP(DT_SLOT_TURN_MM, kP);
    m_leftLeader.config_kI(DT_SLOT_TURN_MM, kI);
    m_leftLeader.config_kD(DT_SLOT_TURN_MM, kD);
    //m_leftLeader.config_kF(DT_SLOT_TURN_MM, kF);

    m_rightLeader.config_kP(DT_SLOT_TURN_MM, kP);
    m_rightLeader.config_kI(DT_SLOT_TURN_MM, kI);
    m_rightLeader.config_kD(DT_SLOT_TURN_MM, kD);
   // m_rightLeader.config_kF(DT_SLOT_TURN_MM, kF);

  }

  public boolean motionMagicDrive(double target_position) {
    double tolerance = 10;
    
    m_rightLeader.set(ControlMode.MotionMagic, target_position, DemandType.AuxPID, m_auxPidTarget);

		double currentPos_R = m_rightLeader.getSelectedSensorPosition();

		return Math.abs(currentPos_R - target_position)< tolerance;
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