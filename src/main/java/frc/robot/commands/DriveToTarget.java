package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DriveToTarget extends Command{
    Supplier<Pose2d> targetPose;
    Pose2d currentTargetPose;
    RobotContainer robotContainer;
    boolean targetExists = false;
    
    public DriveToTarget(RobotContainer robotContainer, Supplier<Pose2d> targetPose) {  
        this.targetPose = targetPose;
        this.robotContainer = robotContainer;
        addRequirements(robotContainer.getDrivebase());
        this.setName("DriveToTarget");
    }

    @Override
    public void initialize() {
        this.currentTargetPose = targetPose.get();
        if (currentTargetPose.equals(new Pose2d())) {
            System.out.println("No target found");
            targetExists = false;
        }
        else {
            System.out.println("Target found at " + currentTargetPose.toString());
            targetExists = true;
        }

        if(targetExists) {
            System.out.println("Driving to target at " + currentTargetPose.toString());
            robotContainer.getDrivebase().driveToPose(currentTargetPose).schedule();
        }
    }

    @Override
    public void execute() {
        System.out.println("im driving");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Done driving to target");
    }
}
