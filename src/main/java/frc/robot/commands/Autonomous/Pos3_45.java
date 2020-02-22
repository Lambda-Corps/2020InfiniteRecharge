/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveMM;
import frc.robot.commands.TurnMM;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Pos3_45 extends SequentialCommandGroup {
  /**
   * Creates a new Pos3_45.
   */
  public Pos3_45(DriveTrain driveTrain) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super();
    addCommands(
      new PrintCommand("shoot 3 balls"),
      new TurnMM(driveTrain, -45), //TODO find this angle
      parallel(
        new DriveMM(driveTrain,-158.13),
        new PrintCommand("drop intake")
      ),
      parallel(
        new DriveMM(driveTrain,158.13),
        new PrintCommand("raise intake")
      ),
      new TurnMM(driveTrain, 45), //find this angle
      new PrintCommand("shoot 3 balls")
    );
  }
}
