package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MoveToTarget extends SequentialCommandGroup {
    
    int aprilTagID;
    Pose2d targetPose;
    SwerveSubsystem swerveSubsystem;

    public MoveToTarget(SwerveSubsystem swerveSubsystem, int aprilTagID) {  
        this.aprilTagID = aprilTagID;
        this.swerveSubsystem = swerveSubsystem;
        targetPose = new Pose2d(3.417, 5.813, Rotation2d.fromDegrees(-60));
        
        addCommands(
            swerveSubsystem.driveToPose(targetPose)
        );
    }

}
