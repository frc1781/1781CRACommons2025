package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;

public class Shoot extends Command
{
  Timer t;
  Lights lights;
  public Shoot(Lights lights)
  {
    t = new Timer();
    this.lights = lights;
    //addRequirements(lights);
  }

  @Override
  public void initialize()
  {
     t.restart();
  }

  @Override
  public void execute()
  {
    lights.run(Lights.Colors.RED, Lights.Patterns.SOLID);
    System.out.println("running lights red");
  }

  @Override
  public boolean isFinished()
  {
    return t.get() > 5;
  }

  @Override
  public void end(boolean interrupted)
  {
    
  }
}

