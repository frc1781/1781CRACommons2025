package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class GroundAlgaePickUp extends SequentialCommandGroup {

    Elevator elevator;
    Arm arm;

    public GroundAlgaePickUp(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
        addCommands(
            new SetAndHoldArm(arm, Arm.ArmState.GROUND_ALGAE),
            new SetAndHoldElevator(elevator, Elevator.ElevatorState.GROUND_COLLECT)
        );
    }
    
}
