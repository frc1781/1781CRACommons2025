package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

public class Conveyor extends SubsystemBase {
    private final SparkMax motor = new SparkMax(Constants.Conveyor.MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();

    public Conveyor() {
        config.idleMode(SparkBaseConfig.IdleMode.kCoast);
        config.smartCurrentLimit(Constants.Conveyor.CURRENT_LIMIT);
        motor.configure(config,
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters
        );
    }
    
    public Command clearCoral(BooleanSupplier hasCoralToClear) {
       
        return new RunCommand(() -> {
            double dc;
            if (hasCoralToClear.getAsBoolean()) {
                dc = 0.5;
            } else {
                dc = 0;
            }
            Logger.recordOutput("Conveyor/motorOutput", dc);
            motor.set(dc);
        }, this);
    }         
}