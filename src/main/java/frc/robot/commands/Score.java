package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Score extends SequentialCommandGroup {

    Arm arm;
    SwerveSubsystem swervedrive;

    public Score(Arm arm, SwerveSubsystem swervedrive) {
        this.arm = arm;
        this.swervedrive = swervedrive;
        addRequirements(arm);
        addCommands(
            new SetArm(arm, Arm.ArmState.L4),
            new MoveBack(swervedrive)
        );
    }
    
}
