package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ScoreLow extends SequentialCommandGroup {

    Arm arm;
    SwerveSubsystem swervedrive;

    public ScoreLow(Arm arm, SwerveSubsystem swervedrive) {
        this.arm = arm;
        this.swervedrive = swervedrive;
        addRequirements(arm, swervedrive);
        addCommands(
            new SetArm(arm, Arm.ArmState.SCORE_MID),
            new WaitCommand(1),
            new MoveBack(swervedrive)
        );
    }
}
