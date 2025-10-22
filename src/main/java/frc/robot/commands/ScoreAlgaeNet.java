package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ScoreAlgaeNet extends ParallelCommandGroup {

    Elevator elevator;
    Arm arm;

    public ScoreAlgaeNet(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
        addCommands(
            new SetAndHoldArm(arm, Arm.ArmState.READY_ALGAE),
            new SetAndHoldElevator(elevator, Elevator.ElevatorState.L4)
        );
    }
    
}
