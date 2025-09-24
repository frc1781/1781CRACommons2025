package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Sensation;

public class Collect extends Command {
  Sensation sensation;
  Elevator elevator;
  Elevator.ElevatorState collectLow = Elevator.ElevatorState.COLLECT_LOW;

  public Collect(Elevator elevator, Sensation sensation) {
    this.elevator = elevator;
    this.sensation = sensation;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    Logger.recordOutput("Elevator/CurrentCommand", "Collect: " + collectLow.name());
    if (sensation.coralExitedHopper()) {
      elevator.setElevatorPosition(collectLow);
    }
  }

  @Override
  public boolean isFinished() {
    return sensation.clawCoralPresent();
  }

  @Override
  public void end(boolean interrupted) {

  }
}
