package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Sensation;

public class PreCollectAuto extends SequentialCommandGroup {

    Elevator elevator;
    Arm arm;

    public PreCollectAuto(Elevator elevator, Arm arm, Sensation sensation) {
        this.elevator = elevator;
        this.arm = arm;
        addRequirements(elevator, arm);
        Logger.recordOutput("RobotContainer/LastRunningCommand", this.getName());
        addCommands(
            new SetElevator(elevator, Elevator.ElevatorState.SAFE_CORAL),
            new SetArm(arm, Arm.ArmState.COLLECT)
        );
    }
    
}
