package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class L2 extends SequentialCommandGroup {

    Elevator elevator;
    Arm arm;

    public L2(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
        Logger.recordOutput("RobotContainer/LastRunningCommand", this.getName());
        addCommands(
            new SetArm(arm, Arm.ArmState.START_MID),
            new SetElevator(elevator, Elevator.ElevatorState.L2),
            new SetArm(arm, Arm.ArmState.WAIT)
        );
    }
    
}
