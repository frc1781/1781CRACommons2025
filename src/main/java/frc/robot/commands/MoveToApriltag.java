package frc.robot.commands;

import java.awt.Robot;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Vision;

public class MoveToApriltag extends Command {

    RobotContainer robotContainer;
    SwerveSubsystem swerveSubsystem;
    Vision visionSubsystem;
    Pose2d targetPose;
    int aprilTagID;
    double distance;

    public MoveToApriltag(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.swerveSubsystem = robotContainer.getDrivebase();
        this.visionSubsystem = swerveSubsystem.getVision();
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

        

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

}
