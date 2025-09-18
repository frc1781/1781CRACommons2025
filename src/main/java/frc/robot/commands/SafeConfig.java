package frc.robot.commands.swervedrive.auto;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class SafeConfig extends Command {

    Elevator elevator;
    SparkMax motor;
    

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
    
}
