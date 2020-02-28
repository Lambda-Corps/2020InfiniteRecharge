/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveMM;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotDistance;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Pos2_90 extends SequentialCommandGroup {
  /**
   * Creates a new Auto2.
   */
  public Pos2_90(DriveTrain driveTrain, Vision vision, Shooter shooter, Intake intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(
      // new RotateToTarget(vision, driveTrain),
      new AutoShootMax4Seconds(shooter, intake, ShotDistance.InitiationLine),
      new DriveMM(driveTrain, -30)
      // new TurnMM(driveTrain, 90),
      // new DriveMM(driveTrain, 58),
      // new TurnMM(driveTrain, -90),
      // parallel(
      //   new DriveMM(driveTrain,-158.13),
      //   new AutoIntakeDown(intake, false)
      // ),
      // parallel(
      //   new DriveMM(driveTrain,158.13),
      //   new AutoIntakeUp(intake)
      // ),
      // new TurnMM(driveTrain, -90),
      // new DriveMM(driveTrain, 58),
      // new TurnMM(driveTrain, 90),
      // new RotateToTarget(vision, driveTrain)
      // new AutoShootMax4Seconds(shooter, intake, ShotDistance.InitiationLine)
    );
  }
}
