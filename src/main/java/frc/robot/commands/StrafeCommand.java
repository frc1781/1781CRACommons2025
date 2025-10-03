package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Sensation;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class StrafeCommand extends Command {
    SwerveSubsystem driveSystem;
    Elevator elevator;
    Arm arm;
    Sensation sensations;
    boolean isLeft;
    boolean isFinished;
    Timer timer;
    
    public StrafeCommand(SwerveSubsystem driveSystem, Elevator elevator, Arm arm, Sensation sensations, boolean isLeft) {
        this.driveSystem = driveSystem;
        this.elevator = elevator;
        this.arm = arm;
        this.sensations = sensations;
        this.isLeft = isLeft;
        timer = new Timer();
        addRequirements(driveSystem);
    }

    public void initialize() {
        isFinished = false;
        timer.start();
    }

    public void execute() {
        if (sensations.armTOFisValid() && sensations.armTOF() < 800) {
            isFinished = true;
        }
        if (timer.get() > 3.0) {
            isLeft = !isLeft;
            timer.reset();
            timer.start();
        }
        ChassisSpeeds requiredSpeeds = new ChassisSpeeds();
        requiredSpeeds.vyMetersPerSecond = isLeft? 0.2 : -0.2;
        driveSystem.drive(requiredSpeeds);
        Logger.recordOutput("Drive/CurrentCommand", "Strafe");
    }

    public boolean isFinished() {
        return isFinished;
    }
}
