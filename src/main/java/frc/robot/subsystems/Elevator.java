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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.EEUtil;
import frc.robot.utils.EEtimeOfFlight;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase{

    private boolean isIdle = true;
    private ElevatorState currentState;
    private SparkMax motorRight;
    private SparkMax motorLeft;
    private double elevatorDutyCycle;
    private RobotContainer robotContainer;

    private EEtimeOfFlight frameTOF;
    private EEtimeOfFlight carriageTOF;

    public double minCarriageDistance = 0;
    public double maxCarriageDistance = 680;

    public double minFrameDistance = 0;
    public double maxFrameDistance = 810; 

    private final double IDLE_DUTY_CYCLE = 0.02;

    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward
    (
        Constants.Elevator.ELEVATOR_KS,
        Constants.Elevator.ELEVATOR_KG,
        Constants.Elevator.ELEVATOR_KV,
        Constants.Elevator.ELEVATOR_KA
    );
    
    private final HashMap<ElevatorState, Double[]> positions = new HashMap<>();
    
    public Elevator(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        elevatorDutyCycle = clampDutyCycle(0);

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

        positions.put(ElevatorState.POLE, new Double[]{750.0, minCarriageDistance});
        positions.put(ElevatorState.SAFE, new Double[]{minFrameDistance, 80.0});
        positions.put(ElevatorState.SAFE_CORAL, new Double[]{minFrameDistance, 40.0});
        positions.put(ElevatorState.L1, new Double[]{0.0, 0.0});
        positions.put(ElevatorState.L2, new Double[]{minFrameDistance, 80.0});
        positions.put(ElevatorState.L3, new Double[]{minFrameDistance, 165.0});
        positions.put(ElevatorState.L3_LOW, new Double[]{minFrameDistance, 350.0});
        positions.put(ElevatorState.L4, new Double[]{maxFrameDistance, minCarriageDistance});
        positions.put(ElevatorState.BARGE_SCORE, new Double[]{maxFrameDistance, minCarriageDistance});
        positions.put(ElevatorState.COLLECT_LOW, new Double[]{minFrameDistance, 260.0});
        positions.put(ElevatorState.GROUND_COLLECT, new Double[]{0.0, 290.0});
        positions.put(ElevatorState.HIGH_ALGAE, new Double[]{minFrameDistance, minCarriageDistance});
        positions.put(ElevatorState.LOW_ALGAE, new Double[]{maxFrameDistance, 350.0});
        positions.put(ElevatorState.SMART_ALGAE, new Double[]{minFrameDistance, 50.0});
    }

    public Command idle() {
        return new InstantCommand(() -> {
            elevatorDutyCycle = IDLE_DUTY_CYCLE;
            Logger.recordOutput("Elevator/CurrentCommand", "Idle");
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
        Logger.recordOutput("Elevator/CurrentState", currentState);
       
        motorRight.set(elevatorDutyCycle);
    }

    public double getFramePosition() {
        return frameTOF.getRange();
    }

    public double getCarriagePosition() {
        return carriageTOF.getRange();
    }

    public boolean hasReachedPosition(ElevatorState desiredState) {
        double carriagePosition = getCarriagePosition();
        double framePosition = getFramePosition();
        double desiredCarriagePosition = positions.get(desiredState)[1];
        double desiredFramePosition = positions.get(desiredState)[0];
        double firstStageDiff = Math.abs(desiredFramePosition - framePosition);
        double secondStageDiff = Math.abs(desiredCarriagePosition - carriagePosition);
        double tolerance = 70;
        return firstStageDiff <= tolerance && secondStageDiff <= tolerance;
    }

    public void setElevatorPosition(ElevatorState desiredState) {
        isIdle = false;
        double carriagePosition = getCarriagePosition();
        double framePosition = getFramePosition();
        double tolerance = 80; // obviously subject to change
        Double[] desiredPosition = positions.get(desiredState);
        System.out.println("hello " + desiredPosition[0] + desiredPosition[1]);
        if (carriageTOF.isRangeValidRegularCheck() && Math.abs(desiredPosition[1] - carriagePosition) >= tolerance) {
            double ff = -feedforwardController.calculate(desiredPosition[1] - carriagePosition);
            Logger.recordOutput("Elevator/FFUnClamped", ff);
            double clampedResult = clampDutyCycle(ff);
            Logger.recordOutput("Elevator/FFClampedOutput", clampedResult);
            elevatorDutyCycle = clampedResult;
        } else if (frameTOF.isRangeValidRegularCheck() && Math.abs(desiredPosition[0] - framePosition) >= tolerance) {
            double ff = feedforwardController.calculate(desiredPosition[0] - framePosition);
            Logger.recordOutput("Elevator/FFUnClamped", ff);
            double clampedResult = clampDutyCycle(ff);
            Logger.recordOutput("Elevator/FFClampedOutput", clampedResult);
            elevatorDutyCycle = clampedResult;
        } else {
            elevatorDutyCycle = IDLE_DUTY_CYCLE;
        }

        // if (((!robotContainer.isSafeForElevatorCarriagetoMove()) && Math.abs(carriagePosition - desiredPosition[1]) > 100)) { //|| !robotController.driveController.isSafeForElevatorStage2toMove()) && Math.abs(secondStagePosition - desiredPosition[1]) > 100) {
        //     elevatorDutyCycle = IDLE_DUTY_CYCLE;
        // }
    }

    public double clampDutyCycle(double dutyCycle) {
        return EEUtil.clamp(0, 0.5, dutyCycle);
    }

    public enum ElevatorState {
        SAFE,
        SAFE_CORAL,
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
