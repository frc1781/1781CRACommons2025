package frc.robot.commands;

import java.util.function.IntSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TargetSide;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MoveToTargetImproved extends SequentialCommandGroup {

    IntSupplier aprilTagID;
    TargetSide sideTargeted;
    RobotContainer robotContainer;
    SwerveSubsystem swerveSubsystem;

    public MoveToTargetImproved(RobotContainer robotContainer) {  
        this.aprilTagID = () -> robotContainer.targetAprilTagID;
        robotContainer.targetedSide = sideTargeted;
        Logger.recordOutput("Drive/CurrentCommand", "RunningDriveToAprilTag:" + aprilTagID.getAsInt() + sideTargeted);
        addCommands(
            new L4(robotContainer.getElevator(), robotContainer.getArm()),
            new DriveToTarget(robotContainer, robotContainer::getTargetPose)
        );
        this.setName("MoveToTargetImproved");   
    }
}
