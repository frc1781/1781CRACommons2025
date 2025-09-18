package frc.robot.subsystems;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.EEtimeOfFlight;

public class Arm extends SubsystemBase {

    private SparkMax armMotor;
    // private HashMap<ArmState,Double> positionMap;
    private EEtimeOfFlight coralTimeOfFlight;
    private Timer timeCoralTOFInvalid;
    private SparkMaxConfig armMotorConfig;

    public Arm() {
        coralTimeOfFlight = new EEtimeOfFlight(Constants.Arm.CLAW_CORAL_SENSOR_ID, 24);
        timeCoralTOFInvalid = new Timer(); 
        armMotor = new SparkMax(Constants.Arm.ARM_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        armMotor.setControlFramePeriodMs(20);
        //New config based on: https://github.com/REVrobotics/2025-REV-ION-FRC-Starter-Bot/blob/main/src/main/java/frc/robot/Configs.java
        armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        armMotorConfig.smartCurrentLimit(40);
        armMotorConfig.absoluteEncoder.positionConversionFactor(360);
        armMotorConfig.absoluteEncoder.zeroOffset(0.4868528);

        // Slot 0 configs
        armMotorConfig.closedLoop.pid(0.004, 0,0.001);
        armMotorConfig.closedLoop.velocityFF((double) 1 /565); // https://docs.revrobotics.com/brushless/neo/vortex#motor-specifications
        armMotorConfig.closedLoop.outputRange(-.55, .55); //(-.55, .55);
        armMotorConfig.closedLoop.positionWrappingEnabled(true);

        armMotorConfig.softLimit.forwardSoftLimit(180);
        armMotorConfig.softLimit.reverseSoftLimit(0);
        armMotorConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {

    }

    public double getPosition() {
        return armMotor.getAbsoluteEncoder().getPosition();
    }

    public void setArmPosition(double position) {

        double FF = -0.095 * Math.sin(Rotation2d.fromDegrees(getPosition()).getRadians());

        // armMotor.getClosedLoopController().setReference(
        //             position,
        //             ControlType.kPosition,
        //             ClosedLoopSlot.kSlot0,
        //             FF,
        //             SparkClosedLoopController.ArbFFUnits.kPercentOut
        //         );

        Logger.recordOutput("Arm/DutyCycle", armMotor.getAppliedOutput());
    }

    public enum ArmState {
        IDLE, L1, L2, L3, L4, MANUAL_UP, MANUAL_DOWN, COLLECT, WAIT, POLE, START_MID, START_HIGH, GROUND_ALGAE, REEF_ALGAE, READY_ALGAE, SLIGHT_TOSS
    }

}
