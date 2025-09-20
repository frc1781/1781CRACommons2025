package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetArm extends Command {

    Arm arm;
    Arm.ArmState desiredState;

    public SetArm(Arm arm, Arm.ArmState desiredState) {
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
        Logger.recordOutput("Arm/currentCommand", "SetArm: " + desiredState.name());
    }

    @Override
    public boolean isFinished() {
        return arm.matchesState();
    }
    
}
