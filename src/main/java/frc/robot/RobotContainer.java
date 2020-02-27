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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.calibration.PIDTuningCommand;
import frc.robot.commands.AlignWithVision;
import frc.robot.commands.ClimbAndLock;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.DefaultDriveTrainCommand;
import frc.robot.commands.Drive_Backwards;
import frc.robot.commands.EditTalonSpeeds;
import frc.robot.commands.ExtendClimberSolenoid;
import frc.robot.commands.RetractClimberSolenoid;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.TurnMM;
import frc.robot.commands.Autonomous.DriveOffLine;
import frc.robot.commands.Autonomous.Pos1;
import frc.robot.commands.Autonomous.Pos2_90;
import frc.robot.commands.Autonomous.Pos3_45;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.calibration.ShooterTuningCommand;
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
  //private final ColorWheel m_cColorWheel;

  // Shuffleboard Info is the container for all the shuffleboard pieces we want to see
  private ShuffleboardInfo m_sbi_instance;
  // The robot's operator interface functionality goes here
  private final XboxController m_driver_controller = new XboxController(DRIVER_REMOTE_PORT);
  private JoystickButton driver_RB, driver_A, /*driver_X, */driver_LB, driver_Start, driver_Back; 

  
  //private DefaultIntakeCommand m_dDefaultIntakeCommand;
  //private Intake m_Intake;

  private SendableChooser<Command> m_auto_chooser;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_auto_chooser = new SendableChooser<Command>();
    m_auto_chooser.addOption("Position 1 Auto", new Pos1(m_drive_train, m_vision, m_shooter, m_intake));
    m_auto_chooser.addOption("Positon 2 Auto", new Pos2_90(m_drive_train, m_vision, m_shooter, m_intake));
    m_auto_chooser.addOption("Position 3 Auto", new Pos3_45(m_drive_train, m_vision, m_shooter, m_intake));
    m_auto_chooser.addOption("Drive Off of the Initiation Line", new DriveOffLine(m_drive_train));

    Shuffleboard.getTab("PID Tuning").add(new ShooterTuningCommand(m_shooter,m_intake)).withWidget(BuiltInWidgets.kCommand);

    // Set the default commands for the subsystems
    m_drive_train.setDefaultCommand(new DefaultDriveTrainCommand(m_drive_train, m_driver_controller));
  

    // Configure the button bindings
    // NOTE -- This should not be called until all the subsystems have been instantiated and the 
    // default commands for them have been set.
    configureButtonBindings();

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
    driver_RB.whenHeld(new Drive_Backwards(m_drive_train, m_driver_controller));
    driver_A = new JoystickButton(m_driver_controller, 1);
    driver_A.whenHeld(new AlignWithVision(m_drive_train, m_vision));
    driver_LB = new JoystickButton(m_driver_controller, XboxController.Button.kBumperLeft.value);
    driver_LB.whenPressed(new ToggleIntake(m_intake));
    driver_Start = new JoystickButton(m_driver_controller,XboxController.Button.kStart.value);
    driver_Start.whenPressed(new ClimberUp(m_climber));
    driver_Back = new JoystickButton(m_driver_controller,XboxController.Button.kBack.value);
    driver_Back.whenPressed(new ClimbAndLock(m_climber));
    


    }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    
    return null;
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
}