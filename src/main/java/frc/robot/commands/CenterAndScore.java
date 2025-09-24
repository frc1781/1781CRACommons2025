package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Sensation;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class CenterAndScore extends SequentialCommandGroup {
    RobotContainer robotContainer;
    Arm arm;
    Elevator elevator;
    Sensation sensation;
    SwerveSubsystem swerveDrive;
    boolean isLeft;
    
    public CenterAndScore(RobotContainer robotContainer, boolean isLeft) {
        this.isLeft = isLeft;
        this.robotContainer = robotContainer;
        this.swerveDrive = robotContainer.getDrivebase();
        this.elevator = robotContainer.getElevator();
        this.arm = robotContainer.getArm();
        this.sensation = robotContainer.getSensation();
        addRequirements(swerveDrive, elevator, arm);
        addCommands(
            new L4(elevator, arm),
            swerveDrive.new MoveToPositionToScore(sensation),
            new StrafeCommand(swerveDrive, elevator, arm, sensation, isLeft),
            new Score(arm, swerveDrive),
            new PreCollect(elevator, arm)
        );

    }
}
