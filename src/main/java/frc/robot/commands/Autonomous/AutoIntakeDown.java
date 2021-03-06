/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoIntakeDown extends CommandBase {
  private final Intake m_intake;
  private final boolean m_conveyorToo;
  private boolean m_isdone;

  public AutoIntakeDown(Intake intake, boolean runConveyor) {
    m_intake = intake;
    m_conveyorToo = runConveyor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isdone = false;
    m_intake.putIntakeDown();
  }

  @Override
  public void execute(){
    if( m_conveyorToo ){
      m_isdone = m_intake.pullInBalls();
    }else {
      m_intake.turnOnIndexerAndIntakeOnly();
    }
  }

  @Override 
  public void end( boolean interrupted ){
    m_intake.stopMotors();
  }

  @Override
  public boolean isFinished(){
    return m_isdone;
  }
}
