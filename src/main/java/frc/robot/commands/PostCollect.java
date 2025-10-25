package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

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
        Logger.recordOutput("RobotContainer/LastRunningCommand", this.getName());
        addCommands(
            new SetElevator(elevator, Elevator.ElevatorState.SAFE_CORAL),
            new SetArm(arm, Arm.ArmState.START_HIGH)
        );
    }
    
}
