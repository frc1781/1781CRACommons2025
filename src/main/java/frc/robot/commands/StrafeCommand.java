package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
    int strafeTries = 0;
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
        strafeTries = 1; //when it's initialized it starts strafing, so this is the first try
    }

    public void execute() {
        if (sensations.armTOFisValid() && sensations.armTOF() < 800) {
            isFinished = true;
        }
        if (timer.get() > 2.0 && strafeTries == 1) {
            isLeft = !isLeft;
            timer.reset();
            timer.start();
            strafeTries = 2;
        }
        if (timer.get() > 3.0 && strafeTries == 2) {
            isLeft = !isLeft;
            timer.reset();
            timer.start();
            CommandScheduler.getInstance().cancelAll();
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
