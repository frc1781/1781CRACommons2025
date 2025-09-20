package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
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

public class SetElevator extends Command
{
  Elevator elevator;
  Elevator.ElevatorState desiredState;

  public SetElevator(Elevator elevator, Elevator.ElevatorState desiredState)
  {
    this.elevator = elevator;
    this.desiredState = desiredState;
    addRequirements(elevator);
  }

  @Override
  public void initialize()
  {

  }

  @Override
  public void execute()
  {
    elevator.setElevatorPosition(desiredState);
    Logger.recordOutput("Elevator/currentCommand", "SetElevator: " + desiredState.name());
  }

  @Override
  public boolean isFinished()
  {
    return elevator.hasReachedPosition(desiredState);
  }
}