package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Sensation;

public class CollectAndClear extends SequentialCommandGroup {

    Elevator elevator;
    Arm arm;
    Sensation sensation;

    public CollectAndClear(Elevator elevator, Arm arm, Sensation sensation) {
        this.elevator = elevator;
        this.arm = arm;
        this.sensation = sensation;
        addRequirements(elevator, arm);
        Logger.recordOutput("RobotContainer/LastRunningCommand", this.getName());
        addCommands(
            new SetElevator(elevator, Elevator.ElevatorState.SAFE_CORAL),
            new Collect(elevator, arm, sensation),
            new PostCollect(elevator, arm)
        );
    }
    
}
