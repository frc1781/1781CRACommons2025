package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class L2hold extends SequentialCommandGroup {

    Elevator elevator;
    Arm arm;

    public L2hold(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
        addCommands(
            new SetArm(arm, Arm.ArmState.START_MID),
            new SetAndHoldElevator(elevator, Elevator.ElevatorState.L2),
            new SetAndHoldArm(arm, Arm.ArmState.WAIT)
        );
    }
    
}
