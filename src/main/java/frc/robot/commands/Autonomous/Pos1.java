/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveMM;
import frc.robot.commands.RotateToTarget;
import frc.robot.commands.ShootFromInitiationLine;
import frc.robot.commands.TurnMM;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Pos1 extends SequentialCommandGroup {
  /**
   * Creates a new Auto1.
   */
  public Pos1(DriveTrain driveTrain, Vision vision, Shooter shooter, Intake intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(//);
    //addCommands(
      // TODO add timeouts to the relevant calls to make sure we don't get stuck
      parallel(
        new DriveMM(driveTrain, -82.86),
        new AutoIntakeDown(intake, false)
      ),
      parallel(
        new DriveMM(driveTrain, 82.86),
        new AutoIntakeUp(intake)
      ),
      new TurnMM(driveTrain, 90).withTimeout(1),
      new DriveMM(driveTrain, 200), 
      new TurnMM(driveTrain, -90),
      new RotateToTarget(vision, driveTrain).withTimeout(.1), // TODO fix after calibrating command
      new ShootFromInitiationLine(shooter, intake).withTimeout(4)
    );
      
    
  }
}
