
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveMM;
import frc.robot.commands.RotateBackToOriginalHeading;
import frc.robot.commands.RotateToTarget;
import frc.robot.commands.TurnOnLimelightLEDs;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotDistance;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Pos3_45 extends SequentialCommandGroup {
  /**
   * Creates a new Pos3_45.
   */
  public Pos3_45(DriveTrain driveTrain, Vision vision, Shooter shooter, Intake intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(
      new DriveMM(driveTrain, -54),
      new TurnOnLimelightLEDs(vision),
      new RotateToTarget(vision, driveTrain).withTimeout(0.75),
      new AutoShootMax7Seconds(shooter, intake, ShotDistance.FrontTrench),
      new RotateBackToOriginalHeading(driveTrain).withTimeout(0.75),
      new ParallelCommandGroup(
        new DriveMM(driveTrain,-105), // TODO test this
        new AutoIntakeDown(intake, true)
      ).withTimeout(5),
      new ParallelCommandGroup(
        new AutoIntakeUp(intake),
        new DriveMM(driveTrain,105)// TODO test this
      ), 
      new RotateToTarget(vision, driveTrain).withTimeout(0.75),
      new AutoShootMax7Seconds(shooter, intake, ShotDistance.FrontTrench)
    );
  }
}
