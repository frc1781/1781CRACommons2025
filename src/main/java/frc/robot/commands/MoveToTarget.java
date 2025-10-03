package frc.robot.commands;

import java.util.function.IntSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.TargetSide;

public class MoveToTarget extends Command {
    
    int aprilTagID;
    TargetSide sideTargeted;
    Pose2d targetPose;
    RobotContainer robotContainer;
    SwerveSubsystem swerveSubsystem;

    public MoveToTarget(RobotContainer robotContainer) {  
        this.robotContainer = robotContainer;
        this.swerveSubsystem = robotContainer.getDrivebase();
        this.aprilTagID = 19;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        this.aprilTagID = robotContainer.targetAprilTagID;
        sideTargeted = robotContainer.targetedSide;
        Logger.recordOutput("Drive/CurrentCommand", "RunningDriveToAprilTag:" + aprilTagID + sideTargeted);
        if(aprilTagID == -1) {
            return;
        } 
        targetPose = robotContainer.scorePose(aprilTagID, sideTargeted);
        swerveSubsystem.driveToPose(targetPose).schedule();
    }

    @Override
    public boolean isFinished() {
        if(aprilTagID == -1) {
            return true;
        } 
        return 
            swerveSubsystem.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.1 && 
            Math.abs(swerveSubsystem.getPose().getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < 5;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Drive/CurrentCommand", "FinishedDriveToAprilTag");
    }
}
