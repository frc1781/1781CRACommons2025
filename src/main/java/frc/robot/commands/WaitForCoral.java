package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sensation;

public class WaitForCoral extends Command {

    Sensation sensation;


    public WaitForCoral(Sensation sensation) {
        this.sensation = sensation;     
    }

    @Override
    public void execute() {
        Logger.recordOutput("Drive/CurrentCommand", "WaitingForCoral");
    }

  @Override
  public boolean isFinished()
  {
    return sensation.coralPresent();
  }

  @Override
  public void end(boolean interrupted)
  {
    
  }

    
}
