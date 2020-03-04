/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.calibration.PIDTuningCommand;
import frc.robot.calibration.ShooterTuningCommand;
import frc.robot.commands.AlignWithVision;
import frc.robot.commands.ClimbAndLock;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.DefaultDriveTrainCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.Drive_Backwards;
import frc.robot.commands.EditTalonSpeeds;
import frc.robot.commands.ExtendClimberSolenoid;
import frc.robot.commands.HoldIntakeDown;
import frc.robot.commands.IntakeCancel;
import frc.robot.commands.RetractClimberSolenoid;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterCancel;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.TurnMM;
import frc.robot.commands.TurnOffLImelightLEDs;
import frc.robot.commands.TurnOnLimelightLEDs;
import frc.robot.commands.Shifting.ToggleShifting;
import frc.robot.commands.Shooter.SetShooterDistance;
import frc.robot.commands.autonomous.DriveOffLine;
import frc.robot.commands.autonomous.Pos1;
import frc.robot.commands.autonomous.BackInitLineShoot3AndDrive;
import frc.robot.commands.autonomous.DriveAndShootFromPortWall;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotDistance;
import frc.robot.subsystems.Vision;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here
  private final DriveTrain m_drive_train = new DriveTrain();
  private final Vision m_vision = new Vision();
  private final Climber m_climber = new Climber();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  //private final ColorWheel m_ColorWheel;

  // Shuffleboard Info is the container for all the shuffleboard pieces we want to see
  private ShuffleboardInfo m_sbi_instance;
  // The robot's operator interface functionality goes here
  private final XboxController m_driver_controller = new XboxController(DRIVER_REMOTE_PORT);
  private JoystickButton driver_RB, driver_A, /*driver_X, */driver_LB, driver_Start, driver_Back, driver_stick_left, driver_stick_right;
  private POVButton driver_POVright, driver_POVbottom; // driver_POVtop,, driver_POVleft; 
  private final XboxController m_partner_controller = new XboxController(PARTNER_REMOTE_PORT);
  private JoystickButton partner_Start, partner_Back, partner_B, partner_A; //partner_LB, partner_RB, partner_X, partner_Y, 
  
  //private DefaultIntakeCommand m_dDefaultIntakeCommand;
  //private Intake m_Intake;

  private SendableChooser<Command> m_auto_chooser;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_auto_chooser = new SendableChooser<Command>();
    m_auto_chooser.addOption("Back of Line, shoot", new BackInitLineShoot3AndDrive(m_drive_train, m_shooter, m_intake));
    m_auto_chooser.addOption("Front of Line, drive to wall", new DriveAndShootFromPortWall(m_drive_train, m_shooter, m_intake));
    // m_auto_chooser.addOption("Position 1 Auto", new Pos1(m_drive_train, m_vision, m_shooter, m_intake));
    // m_auto_chooser.addOption("Positon 2 Auto", new Pos2_90(m_drive_train, m_vision, m_shooter, m_intake));
    //m_auto_chooser.addOption("Position 3 Auto", new Pos3_45(m_drive_train, m_vision, m_shooter, m_intake));
    m_auto_chooser.addOption("Drive Off of the Initiation Line", new DriveOffLine(m_drive_train));
    

    Shuffleboard.getTab("PID Tuning").add(new ShooterTuningCommand(m_shooter,m_intake)).withWidget(BuiltInWidgets.kCommand);

    // Set the default commands for the subsystems
    m_drive_train.setDefaultCommand(new DefaultDriveTrainCommand(m_drive_train, m_driver_controller));
    m_intake.setDefaultCommand(new DefaultIntakeCommand(m_intake, m_partner_controller));
  

    // Configure the button bindings
    // NOTE -- This should not be called until all the subsystems have been instantiated and the 
    // default commands for them have been set.
    configureButtonBindings();
    // SmartDashboard.putData(new DriveMM(m_drive_train, -83));
    // SmartDashboard.putData(new TurnMM(m_drive_train, 90));

    setupShuffleBoard();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driver_RB = new JoystickButton(m_driver_controller, XboxController.Button.kBumperRight.value);
    driver_RB.whileHeld(new Drive_Backwards(m_drive_train, m_driver_controller));
    driver_A = new JoystickButton(m_driver_controller, 1);
    driver_A.whileHeld(new AlignWithVision(m_drive_train, m_vision));
    driver_LB = new JoystickButton(m_driver_controller, XboxController.Button.kBumperLeft.value);
    driver_LB.whileHeld(new HoldIntakeDown(m_intake));
    driver_Start = new JoystickButton(m_driver_controller,XboxController.Button.kStart.value);
    driver_Start.whenPressed(new ClimberUp(m_climber));
    driver_Back = new JoystickButton(m_driver_controller,XboxController.Button.kBack.value);
    driver_Back.whenPressed(new ClimbAndLock(m_climber));
    driver_stick_left = new JoystickButton(m_driver_controller, XboxController.Button.kStickLeft.value);
    driver_stick_left.whenPressed(new TurnOffLImelightLEDs(m_vision));
    driver_stick_right = new JoystickButton(m_driver_controller, XboxController.Button.kStickRight.value);
    driver_stick_right.whenPressed(new TurnOnLimelightLEDs(m_vision));
    // driver_r_jb = new JoystickButton( m_driver_controller, XboxController.Button.kStickRight.value);
    // driver_POVtop = new POVButton(m_driver_controller, 0);
    // driver_POVtop.whenPressed(new SetShooterDistance(m_shooter, ShotDistance.FrontTrench));
    driver_POVright = new POVButton(m_driver_controller, 90);
    driver_POVright.whenPressed(new SetShooterDistance(m_shooter, ShotDistance.InitiationLine));
    driver_POVbottom = new POVButton( m_driver_controller, 180);
    driver_POVbottom.whenPressed(new SetShooterDistance(m_shooter, ShotDistance.PortWall));
    // driver_POVleft = new POVButton(m_driver_controller, 270);
    
    //jb= joystick button

    // Partner controls
    partner_Back = new JoystickButton(m_partner_controller, XboxController.Button.kBack.value);
    partner_Back.whenPressed(new IntakeCancel(m_intake));

    partner_Start = new JoystickButton(m_partner_controller, XboxController.Button.kStart.value);
    partner_Start.whenPressed(new ShooterCancel(m_shooter));

    // partner_X = new JoystickButton(m_partner_controller, XboxController.Button.kX.value);
    // partner_X.whenPressed(new );
    // partner_Y = new JoystickButton(m_partner_controller, XboxController.Button.kY.value);
    // partner_Y.whenPressed(new Shoot(m_shooter, m_intake, ShotDistance.FrontTrench));
    partner_B = new JoystickButton(m_partner_controller, XboxController.Button.kB.value);
    partner_B.whenPressed(new Shoot(m_shooter, m_intake, ShotDistance.InitiationLine));
    partner_A = new JoystickButton(m_partner_controller, XboxController.Button.kA.value);
    partner_A.whileHeld(new Shoot(m_shooter, m_intake, ShotDistance.PortWall));
    // partner_Start = new JoystickButton(m_partner_controller, XboxController.Button.kStart.value);
    // partner_Start.whenPressed(new Spin3Times(m_colorwheel));
    // partner_Back = new JoystickButton(m_partner_controller, XboxController.Button.kBack.value);
    // partner_Back.whenPressed(new SpinToAColor(m_colorwheel));
    // partner_LB = new JoystickButton(m_partner_controller, XboxController.Button.kBumperLeft.value);
    // partner_LB.whileHeld(new );
    // partner_RB = new JoystickButton(m_partner_controller, XboxController.Button.kBumperRight.value);
    // partner_RB.whileHeld(new );



  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    
    return m_auto_chooser.getSelected();
  }

  private void setupShuffleBoard(){
    // Setup methods for each subset of Shuffleboard needed setup
    m_sbi_instance = ShuffleboardInfo.getInstance();
    // Setup Autonomous
    m_sbi_instance.addAutoChooser(m_auto_chooser);
    
    setupClimberShuffleBoard();
  }

  @SuppressWarnings("unused")
  private void setupAutonomousShuffleboard(){
    SmartDashboard.putData("Autonomous", new Pos1(m_drive_train, m_vision, m_shooter, m_intake));
    
  }

  @SuppressWarnings("unused")
  private void setupPidTuningCommandShuffleboard(){
    // First, assign a local variable the Tab that we are going to use
    // for pid tuning in Shuffleboard
    Shuffleboard.getTab("PID Tuning").add(new PIDTuningCommand());
  }

  @SuppressWarnings("unused")
  private void setUpTalonSpeedCommand(){
    Shuffleboard.getTab("Talon Tuning").add(new EditTalonSpeeds(m_drive_train));
  }

  @SuppressWarnings("unused")
  private void setupClimberShuffleBoard(){
    Shuffleboard.getTab("Climber").add("up",new ClimberUp(m_climber));
    Shuffleboard.getTab("Climber").add("down",new ClimberDown(m_climber));
    Shuffleboard.getTab("Climber").add("Lift up and fire Solenoid",new ClimbAndLock(m_climber));
    Shuffleboard.getTab("Climber").add("ExtendSolenoid",new ExtendClimberSolenoid(m_climber));
    Shuffleboard.getTab("Climber").add("RetractSolnoid",new RetractClimberSolenoid(m_climber));
  }
  
  @SuppressWarnings("unused")
  private void setupDriveMMShuffleboard(){
    // First, assign a local variable the Tab that we are going to use
    // for pid tuning in Shuffleboard
    
    //Shuffleboard.getTab("Drive MM").add(new DriveMM(m_drive_train, 100)).withPosition(0, 0);
    // Shuffleboard.getTab("Drive MM").add(new DriveMM(m_drive_train, -100)).withPosition(2, 0);
    //SmartDashboard.putData("Drive 100", new DriveMM(m_drive_train, 100));
    //SmartDashboard.putData("Drive -100", new DriveMM(m_drive_train, -100));
  }

  @SuppressWarnings("unused")
  private void setupTurnMMShuffleboard(){
    Shuffleboard.getTab("Turn MM Testing").add(new TurnMM(m_drive_train, 0));
  }

  public void setDriveTrainToLowGear(){
    m_drive_train.setLowGear();
  }

  public void stopAllMotors() {
    m_drive_train.stopMotors();
    m_intake.stopMotors();
    m_shooter.stopMotors();
    m_climber.stopMotor();
  }

  public void turnOffLimelightLED(){
    m_vision.setLlLedMode(VISION_LED_OFF);
  }
}