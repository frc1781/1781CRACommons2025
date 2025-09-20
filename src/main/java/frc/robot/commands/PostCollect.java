package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class PostCollect extends SequentialCommandGroup {

    Elevator elevator;
    Arm arm;

    public PostCollect(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
        addRequirements(elevator, arm);
        addCommands(
            new SetElevator(elevator, Elevator.ElevatorState.SAFE),
            new SetArm(arm, Arm.ArmState.START_HIGH)
        );
    }
    
}
