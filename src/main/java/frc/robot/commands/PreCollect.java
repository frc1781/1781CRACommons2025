package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Sensation;

public class PreCollect extends SequentialCommandGroup {

    Elevator elevator;
    Arm arm;

    public PreCollect(Elevator elevator, Arm arm, Sensation sensation) {
        this.elevator = elevator;
        this.arm = arm;
        addRequirements(elevator, arm);
        addCommands(
            new SetElevator(elevator, Elevator.ElevatorState.SAFE_CORAL),
            new SetArm(arm, Arm.ArmState.COLLECT),
            new WaitUntilCommand(() -> false)
        );
    }
    
}
