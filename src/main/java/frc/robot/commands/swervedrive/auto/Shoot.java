package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Lights;

public class Shoot extends ParallelDeadlineGroup
{
  public Shoot(Lights leds) {
    super(
      new WaitCommand(2.0),
      leds.set(Lights.Colors.RED, Lights.Patterns.SOLID)
    );
  }
}
