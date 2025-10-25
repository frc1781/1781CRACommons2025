package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class L4 extends ParallelCommandGroup {

    Elevator elevator;
    Arm arm;

    public L4(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
        Logger.recordOutput("RobotContainer/LastRunningCommand", this.getName());
        addCommands(
            new SetElevator(elevator, Elevator.ElevatorState.L4),
            new SetArm(arm, Arm.ArmState.POLE)
        );
    }
    
}
