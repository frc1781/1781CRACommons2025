package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetAndHoldArm extends Command {

    Arm arm;
    Arm.ArmState desiredState;

    public SetAndHoldArm(Arm arm, Arm.ArmState desiredState) {
        this.arm = arm;
        this.desiredState = desiredState;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        arm.setState(desiredState);
        Logger.recordOutput("Arm/CurrentCommand", "SetAndHoldArm: " + desiredState.name());
    }

    @Override
    public boolean isFinished() {  //hold this command open until interrupted
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Arm/CurrentCommand", "FinishedSetArm: " + desiredState.name());
    }
    
}
