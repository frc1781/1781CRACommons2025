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

public class Collect extends Command
{
  BooleanSupplier coralPresentInArm;
  Elevator elevator;
  Elevator.ElevatorState collectLow = Elevator.ElevatorState.COLLECT_LOW;

  public Collect(Elevator elevator, BooleanSupplier coralPresentInArm)  {
    this.elevator = elevator;
    this.coralPresentInArm = coralPresentInArm;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    elevator.setElevatorPosition(collectLow);
    Logger.recordOutput("Elevator/currentCommand", "Collect: " + collectLow.name());
  }

  @Override
  public boolean isFinished()
  {
    return coralPresentInArm.getAsBoolean() || elevator.hasReachedPosition(collectLow);
  }

  @Override
  public void end(boolean interrupted)
  {
    // if (interrupted) System.out.println("Collect interrupted");
    // else System.out.println("Collect finished");
  }
}

