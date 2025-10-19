package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

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
import frc.robot.commands.SetElevator;

public class Elevator extends SubsystemBase{

    private boolean isIdle = true;
    private ElevatorState currentState;
    private SparkMax motorRight;
    private SparkMax motorLeft;
    private double elevatorDutyCycle;
    private RobotContainer robotContainer;

    private EEtimeOfFlight frameTOF;
    private EEtimeOfFlight carriageTOF;

    public double minCarriageDistance = 120;
    public double maxCarriageDistance = 605;

    public double minFrameDistance = 0;
    public double maxFrameDistance = 730; 

    private final double IDLE_DUTY_CYCLE = 0.02;

    private PIDController pidController = new PIDController(0.005, 0, 0);

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

        positions.put(ElevatorState.POLE, new Double[]{730.0, minCarriageDistance});
        positions.put(ElevatorState.SAFE, new Double[]{minFrameDistance, 150.0});
        positions.put(ElevatorState.SAFE_CORAL, new Double[]{minFrameDistance, 150.0});
        positions.put(ElevatorState.L1, new Double[]{0.0, 0.0});
        positions.put(ElevatorState.L2, new Double[]{minFrameDistance, 540.0});
        positions.put(ElevatorState.L3, new Double[]{60.0, 150.0});
        positions.put(ElevatorState.L3_LOW, new Double[]{minFrameDistance, 350.0});
        positions.put(ElevatorState.L4, new Double[]{maxFrameDistance, minCarriageDistance});
        positions.put(ElevatorState.BARGE_SCORE, new Double[]{maxFrameDistance, minCarriageDistance});
        positions.put(ElevatorState.COLLECT_LOW, new Double[]{minFrameDistance, 340.0});
        positions.put(ElevatorState.GROUND_COLLECT, new Double[]{0.0, 290.0});
        positions.put(ElevatorState.HIGH_ALGAE, new Double[]{minFrameDistance, minCarriageDistance});
        positions.put(ElevatorState.LOW_ALGAE, new Double[]{maxFrameDistance, 350.0});
        positions.put(ElevatorState.SMART_ALGAE, new Double[]{minFrameDistance, 50.0});
    }
    
    public Command idle(BooleanSupplier isArmInsideElevator, BooleanSupplier clawCoralPresent) {
        return new InstantCommand(() -> {
            if(isArmInsideElevator.getAsBoolean()) {
                Logger.recordOutput("Elevator/CurrentCommand", "Waiting for arm to clear");
                elevatorDutyCycle = IDLE_DUTY_CYCLE;
            } else if (clawCoralPresent.getAsBoolean() && !hasReachedPosition(ElevatorState.L4)) {
                new SetElevator(this, ElevatorState.L3).schedule();
            } else {
                new SetElevator(this, ElevatorState.SAFE_CORAL).schedule();
            }
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
        Logger.recordOutput("Elevator/hasReachedSafePosition", hasReachedPosition(ElevatorState.SAFE));
        
        if (currentState == ElevatorState.STOP) {
            elevatorDutyCycle = 0;
        }

        motorRight.set(elevatorDutyCycle);
    }

    public void testPositiveDutyCycle() {
        elevatorDutyCycle = 0.05;
        motorRight.set(elevatorDutyCycle);
    }

    public void testNegativeDutyCycle() {
        elevatorDutyCycle = -0.05;
        motorRight.set(elevatorDutyCycle);
    }

    public double getFramePosition() {
        return frameTOF.getRange();
    }

    public double getCarriagePosition() {
        return carriageTOF.getRange();
    }

    public boolean hasReachedPosition(ElevatorState desiredState) {
        
        if(desiredState == ElevatorState.STOP) {
            return true;
        }
        double carriagePosition = getCarriagePosition();
        double framePosition = getFramePosition();
        double desiredCarriagePosition = positions.get(desiredState)[1];
        double desiredFramePosition = positions.get(desiredState)[0];
        double firstStageDiff = Math.abs(desiredFramePosition - framePosition);
        double secondStageDiff = Math.abs(desiredCarriagePosition - carriagePosition);
        double tolerance = 70;  //CONSIDER TIGHTER TOLERANCE BUT WORKING NOW SO ???
        return firstStageDiff <= tolerance && secondStageDiff <= tolerance;
    }

    public void setElevatorPosition(ElevatorState desiredState) {

        if(desiredState == ElevatorState.STOP) {
            return;
        }

        if (positions.get(desiredState)[0] >= 1 && !robotContainer.getSensation().clawCoralPresent()) {
            System.out.println("Elevator cannot reach state " + desiredState + " because coral is not present in claw");
            System.out.println(positions.get(desiredState)[0]);
            System.out.println(robotContainer.getSensation().clawCoralPresent());
            return;
        }

        isIdle = false;
        double carriagePosition = getCarriagePosition();
        double framePosition = getFramePosition();
        double tolerance = 10; 
        Double[] desiredPosition = positions.get(desiredState);
        if (carriageTOF.isRangeValidRegularCheck() && Math.abs(desiredPosition[1] - carriagePosition) >= tolerance) {
            //double pidDC = pidController.calculate(carriagePosition, desiredPosition[1]);
            double calculatedDC = -0.005 * (desiredPosition[1] - carriagePosition); //simple P only pid no need to call function
            Logger.recordOutput("Elevator/calculatedDC", calculatedDC);
            double clampedResult = clampDutyCycle(calculatedDC);
            Logger.recordOutput("Elevator/clampedDC", clampedResult);
            elevatorDutyCycle = clampedResult;
        } else if (frameTOF.isRangeValidRegularCheck() && Math.abs(desiredPosition[0] - framePosition) >= tolerance) {
            //double pidDC = pidController.calculate(framePosition, desiredPosition[0]);
            double calculatedDC = 0.005 * (desiredPosition[0] - framePosition); //simple P only pid no need to call function
            Logger.recordOutput("Elevator/calculatedDC", calculatedDC);
            double clampedResult = clampDutyCycle(calculatedDC);
            Logger.recordOutput("Elevator/clampedDC", clampedResult);
            elevatorDutyCycle = clampedResult;
        } else {
            elevatorDutyCycle = IDLE_DUTY_CYCLE;
        }
    }

    public double clampDutyCycle(double dutyCycle) {
        return EEUtil.clamp(-0.2, 0.2, dutyCycle);  //needs to be greatly increased but be carefull
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
        BARGE_SCORE,
        STOP
    }
}
