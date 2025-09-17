package frc.robot.subsystems;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.EEUtil;
import frc.robot.utils.EEtimeOfFlight;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Robot;

public class Elevator extends SubsystemBase{

    private boolean isIdle = true;
    private ElevatorState currentState;
    private SparkMax motorRight;
    private SparkMax motorLeft;
    private double elevatorDutyCycle;

    private EEtimeOfFlight frameTOF;
    private EEtimeOfFlight carriageTOF;

    private double minCarriageDistance = 0;
    private double maxCarriageDistance = 680;

    private double minFrameDistance = 0;
    private double maxFrameDistance = 810; 

    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward
            (
                Constants.Elevator.ELEVATOR_KS,
                Constants.Elevator.ELEVATOR_KG,
                Constants.Elevator.ELEVATOR_KV,
                Constants.Elevator.ELEVATOR_KA
            );
    private PIDController positionPID = new PIDController(0.001, 0,0);

    private final HashMap<ElevatorState, Double[]> positions = new HashMap<>();
    
    public Elevator() {
        elevatorDutyCycle = clampDutyCycle(feedforwardController.calculate(0));

        frameTOF = new EEtimeOfFlight(Constants.Elevator.FRAME_TOF, 20);
        carriageTOF = new EEtimeOfFlight(Constants.Elevator.CARRIAGE_TOF, 20);

        //Right Elevator Motor
        motorRight = new SparkMax(Constants.Elevator.RIGHT_ELEVATOR_MOTOR, MotorType.kBrushless);
        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.idleMode(IdleMode.kCoast);
        rightMotorConfig.smartCurrentLimit(30);
        motorRight.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Left Elevator Motor (Follows Right Elevator Motor)
        motorLeft = new SparkMax(Constants.Elevator.LEFT_ELEVATOR_MOTOR, MotorType.kBrushless);
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.idleMode(IdleMode.kCoast);
        leftMotorConfig.follow(motorRight, true);
        leftMotorConfig.smartCurrentLimit(30);
        motorLeft.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // positions.put(ElevatorState.POLE, new Double[]{750.0, minSecondStageDistance});
        // positions.put(ElevatorState.SAFE, new Double[]{minFirstStageDistance, 80.0});
        // positions.put(ElevatorState.L1, new Double[]{0.0, 0.0});
        // positions.put(ElevatorState.L2, new Double[]{minFirstStageDistance, 80.0});
        // positions.put(ElevatorState.L3, new Double[]{minFirstStageDistance, 165.0});
        // positions.put(ElevatorState.L3_LOW, new Double[]{minFirstStageDistance, 350.0});
        // positions.put(ElevatorState.L4, new Double[]{maxFirstStageDistance, minSecondStageDistance});
        // positions.put(ElevatorState.BARGE_SCORE, new Double[]{maxFirstStageDistance, minSecondStageDistance});
        // positions.put(ElevatorState.COLLECT_LOW, new Double[]{minFirstStageDistance, 400.0});
        // positions.put(ElevatorState.GROUND_COLLECT, new Double[]{0.0, 290.0});
        // positions.put(ElevatorState.HIGH_ALGAE, new Double[]{minFirstStageDistance, minSecondStageDistance});
        // positions.put(ElevatorState.LOW_ALGAE, new Double[]{maxFirstStageDistance, 350.0});
        // positions.put(ElevatorState.SMART_ALGAE, new Double[]{minFirstStageDistance, 50.0});
    }

    public Command idle() {
        return new InstantCommand(() -> {
            isIdle = true;
        }, this);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Elevator/FrameTOF", frameTOF.getRange());
        Logger.recordOutput("Elevator/CarriageTOF", carriageTOF.getRange());
        Logger.recordOutput("Elevator/FrameTOFvalid", frameTOF.isRangeValidRegularCheck());
        Logger.recordOutput("Elevator/CarriageTOFvalid", carriageTOF.isRangeValidRegularCheck());
        Logger.recordOutput("Elevator/ElevatorMotorEncoderCounts", motorRight.getEncoder().getPosition());
        Logger.recordOutput("Elevator/DutyCycle", elevatorDutyCycle);

        motorRight.set(elevatorDutyCycle);
    }

    public double getFramePosition() {
        return frameTOF.getRange();
    }

    public double getCarriagePosition() {
        return carriageTOF.getRange();
    }

    public ElevatorState getCurrentState() {
        return currentState;
    }

    public void setElevatorPosition(double desiredPosition) {
        double tolerance = 80; // obviously subject to change
        if (Math.abs(desiredPosition - ((maxCarriageDistance - getCarriagePosition()) + getFramePosition())) >= tolerance) {
            elevatorDutyCycle = clampDutyCycle(feedforwardController.calculate(desiredPosition - ((maxCarriageDistance - getCarriagePosition()) + getFramePosition())));
        }
    }

    public double clampDutyCycle(double dutyCycle) {
        return EEUtil.clamp(0, 0.5, dutyCycle);
    }

    public enum ElevatorState {
        SAFE,
        L1,
        L2,
        L3,
        L3_LOW,
        L4,
        MANUAL_DOWN,
        MANUAL_UP,
        COLLECT_LOW,
        POLE,
        GROUND_COLLECT,
        HIGH_ALGAE,
        LOW_ALGAE,
        SMART_ALGAE,
        BARGE_SCORE
    }

}
