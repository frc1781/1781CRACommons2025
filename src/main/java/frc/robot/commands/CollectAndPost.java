package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Sensation;

public class CollectAndPost extends SequentialCommandGroup {

    Elevator elevator;
    Arm arm;
    Sensation sensation;

    public CollectAndPost(Elevator elevator, Arm arm, Sensation sensation) {
        this.elevator = elevator;
        this.arm = arm;
        this.sensation = sensation;
        addRequirements(elevator, arm);
        addCommands(
            new Collect(elevator, arm, sensation),
            new PostCollect(elevator, arm)
        );
    }

}
