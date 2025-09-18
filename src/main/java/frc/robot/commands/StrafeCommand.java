package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class StrafeCommand extends Command {


    
    SwerveSubsystem driveSystem;
    Elevator elevator;
    Arm arm;

    public StrafeCommand(SwerveSubsystem driveSystem, Elevator elevator, Arm arm) {
        this.driveSystem = driveSystem;
        this.elevator = elevator;
        this.arm = arm;
        addRequirements(driveSystem, elevator, arm);
    }

    public void initialize() {

    }

    public void execute() {

    }

    public boolean isFinished() {
        return true;
    }
}
