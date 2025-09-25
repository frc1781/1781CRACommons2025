package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;

public class MoveToTarget extends SequentialCommandGroup {
    
    int aprilTagID;
    Pose2d targetPose;
    SwerveSubsystem swerveSubsystem;

    public MoveToTarget(SwerveSubsystem swerveSubsystem, int aprilTagID) {  
        this.aprilTagID = aprilTagID;
        this.swerveSubsystem = swerveSubsystem;
        targetPose = Constants.Positions.getPositionForRobot(aprilTagID);
        
        addCommands(
            swerveSubsystem.driveToPose(targetPose)
        );
    }

}
