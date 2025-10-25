package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Sensation;

public class Collecting extends SequentialCommandGroup {

    Elevator elevator;
    Arm arm;

    public Collecting(Elevator elevator, Arm arm, Sensation sensation) {
        this.elevator = elevator;
        this.arm = arm;
        addRequirements(elevator, arm);
        Logger.recordOutput("RobotContainer/LastRunningCommand", this.getName());
        addCommands(
            new PreCollect(elevator, arm, sensation),
            new Collect(elevator, arm, sensation)
        );
    }  
}
