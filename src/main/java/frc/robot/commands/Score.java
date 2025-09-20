package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class Score extends SequentialCommandGroup {

    Arm arm;

    public Score(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
        addCommands(
            new SetArm(arm, Arm.ArmState.SCORE)
        );
    }
    
}
