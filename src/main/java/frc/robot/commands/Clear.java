package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm.ArmState;

public class Clear extends Command {

    boolean isArmClear;
    Arm arm;
    Elevator elevator;

    double armMinDegrees;
    double requiredArmPositionDegrees;

    public Clear(Arm arm, Elevator elevator) {
        this.arm = arm;
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        isArmClear = false;
    }

    @Override
    public void execute() {
        if (RobotContainer.isArmInsideElevator()) {
            arm.setState(ArmState.START_MID);
        }
    }

    @Override
    public boolean isFinished() {
        return arm.matchesState();
    }
    
}
