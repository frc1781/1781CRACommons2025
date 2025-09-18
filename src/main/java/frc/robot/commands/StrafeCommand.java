package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Sensation;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class StrafeCommand extends Command {
    SwerveSubsystem driveSystem;
    Elevator elevator;
    Arm arm;
    Sensation sensations;
    boolean isLeft;
    boolean isFinished;
    
    public StrafeCommand(SwerveSubsystem driveSystem, Elevator elevator, Arm arm, Sensation sensations, boolean isLeft) {
        this.driveSystem = driveSystem;
        this.elevator = elevator;
        this.arm = arm;
        this.sensations = sensations;
        this.isLeft = isLeft;
        addRequirements(driveSystem);
    }

    public void execute() {
        if (sensations.armTOFisValid() && sensations.armTOF() < 80) {
            isFinished = true;
        }
        ChassisSpeeds requiredSpeeds = new ChassisSpeeds();
        requiredSpeeds.vyMetersPerSecond = isLeft? -0.2 : 0.2;
        driveSystem.drive(requiredSpeeds);
    }

    public boolean isFinished() {
        return isFinished;
    }
}
