package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// public class Shoot extends ParallelDeadlineGroup
// {
//   public Shoot(Lights leds) {
//     super(
//       new WaitCommand(5.0),
//       leds.set(Lights.Colors.RED, Lights.Patterns.SOLID)
//     );
//   }
// }



public class Position extends Command
{
  Timer t;
  SwerveSubsystem swerve;
  BooleanSupplier coralPresent;
  

  public Position(SwerveSubsystem swerve, BooleanSupplier coralPresent)
  {
    t = new Timer();
    this.swerve = swerve;
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
    //lights.run(Lights.Colors.RED, Lights.Patterns.SOLID);
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

