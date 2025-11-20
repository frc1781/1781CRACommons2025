package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class L3 extends SequentialCommandGroup {

    Elevator elevator;
    Arm arm;

    public L3(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
        addCommands(
            new SetArm(arm, Arm.ArmState.START_MID),
            new SetElevator(elevator, Elevator.ElevatorState.L3),
            new SetArm(arm, Arm.ArmState.WAIT)
        );
    }  
}
