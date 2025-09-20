package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class L2 extends ParallelCommandGroup {

    Elevator elevator;
    Arm arm;

    public L2(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
        addCommands(
            new SetElevator(elevator, Elevator.ElevatorState.L2),
            new SetArm(arm, Arm.ArmState.L2)
        );
    }
    
}
