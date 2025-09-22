package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MoveBack extends Command{

    SwerveSubsystem swervedrive;
    Timer t;
    ChassisSpeeds requiredSpeeds;

    public MoveBack(SwerveSubsystem swervedrive) {
        this.swervedrive = swervedrive;
        addRequirements(swervedrive);
        t = new Timer();
    }

    @Override
    public void initialize() {
        t.restart();
        requiredSpeeds = new ChassisSpeeds(-0.6, 0, 0);
    }

    @Override
    public void execute() {
        swervedrive.drive(requiredSpeeds);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        t.stop();
    }
}
