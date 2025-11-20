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
            new SetAndHoldArm(arm, Arm.ArmState.SCORE_MID).alongWith(new WaitCommand(0.5).andThen(new MoveBack(swervedrive)))
            // makes sure to HOLD the arm position while moving back (bad code, we can do it later)
        );
    }
}
