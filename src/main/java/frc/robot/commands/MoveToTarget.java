package frc.robot.commands;

import java.util.function.IntSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class MoveToTarget extends Command {
    
    IntSupplier aprilTagID;
    Pose2d targetPose;
    RobotContainer robotContainer;
    SwerveSubsystem swerveSubsystem;

    public MoveToTarget(RobotContainer robotContainer, IntSupplier aprilTagID) {  
        this.robotContainer = robotContainer;
        this.swerveSubsystem = robotContainer.getDrivebase();
        this.aprilTagID = aprilTagID;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Drive/CurrentCommand", "RunningDriveToAprilTag:" + aprilTagID.getAsInt());
        targetPose = Constants.Positions.getPositionForRobot(aprilTagID.getAsInt());
        swerveSubsystem.driveToPose(targetPose).schedule();
    }

    @Override
    public boolean isFinished() {
        return 
            swerveSubsystem.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.1 && 
            Math.abs(swerveSubsystem.getPose().getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < 5;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Drive/CurrentCommand", "FinishedDriveToAprilTag");
    }
}
