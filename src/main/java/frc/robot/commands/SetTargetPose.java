package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetTargetPose extends Command {

    int targetAprilTagID;
    RobotContainer robotContainer;

    public SetTargetPose(RobotContainer robotContainer, int targetAprilTagID) {
        this.targetAprilTagID = targetAprilTagID;        
        this.robotContainer = robotContainer;
    }

    @Override
    public void initialize() {
        robotContainer.setTargetPose(targetAprilTagID);
    }

    @Override
    public boolean isFinished() {
        return true; // Command completes immediately after setting the target pose
    }    
}
