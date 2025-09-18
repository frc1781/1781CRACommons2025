package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

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
        if(arm.getPosition() > armMinDegrees) {                     //greater than the minimum value to consider the arm cleared
            isArmClear = true;
        }
        arm.setArmPosition(requiredArmPositionDegrees);
    }

    @Override
    public boolean isFinished() {
        return isArmClear;
    }
    
}
