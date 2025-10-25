package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ScoreL4 extends SequentialCommandGroup {

    Arm arm;
    SwerveSubsystem swervedrive;

    public ScoreL4(Arm arm, SwerveSubsystem swervedrive) {
        this.arm = arm;
        this.swervedrive = swervedrive;
        addRequirements(arm, swervedrive);
        Logger.recordOutput("RobotContainer/LastRunningCommand", this.getName());
        addCommands(
            new SetArm(arm, Arm.ArmState.SCORE_L4),
            new MoveBack(swervedrive)
        );
    }
    
}
