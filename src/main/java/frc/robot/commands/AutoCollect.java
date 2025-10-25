package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Sensation;

public class AutoCollect extends ParallelCommandGroup{
    public AutoCollect(Elevator elevator, Conveyor conveyor, Arm arm, Sensation sensation) {
        addRequirements(elevator, conveyor, arm);
        addCommands(
            new Collect(elevator, arm, sensation),
            conveyor.clearCoral(()->true, elevator)
        );
    }
}
