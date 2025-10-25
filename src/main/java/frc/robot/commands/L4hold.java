package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class L4hold extends ParallelCommandGroup {

    Elevator elevator;
    Arm arm;

    public L4hold(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
        Logger.recordOutput("RobotContainer/LastRunningCommand", this.getName());
        addCommands(
            new SetAndHoldElevator(elevator, Elevator.ElevatorState.L4),
            new SetAndHoldArm(arm, Arm.ArmState.POLE)
        );
    }
    
}
