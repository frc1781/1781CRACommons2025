package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class L4hold extends ParallelCommandGroup {

    Elevator elevator;
    Arm arm;

    public L4hold(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
        addCommands(
            new SetAndHoldElevator(elevator, Elevator.ElevatorState.L4),
            new SetAndHoldArm(arm, Arm.ArmState.POLE)
        );
    }
    
}
