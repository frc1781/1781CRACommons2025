package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;

// public class Shoot extends ParallelDeadlineGroup
// {
//   public Shoot(Lights leds) {
//     super(
//       new WaitCommand(5.0),
//       leds.set(Lights.Colors.RED, Lights.Patterns.SOLID)
//     );
//   }
// }

public class Collect extends Command
{
  Timer t;
  Lights lights;
  BooleanSupplier coralPresent;

  public Collect(Lights lights, BooleanSupplier coralPresent)
  {
    t = new Timer();
    this.lights = lights;
    this.coralPresent = coralPresent;
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
  }

  @Override
  public boolean isFinished()
  {
    return t.get() > 15 || coralPresent.getAsBoolean();
  }

  @Override
  public void end(boolean interrupted)
  {
    
  }
}

