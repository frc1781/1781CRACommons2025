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
    private SparkMax motorRight;
    private SparkMax motorLeft;
    public double elevatorDutyCycle;
    private RobotContainer robotContainer;

    private EEtimeOfFlight frameTOF;
    private EEtimeOfFlight carriageTOF;

    public double minCarriageDistance = 115;
    public double maxCarriageDistance = 605;

    public double minFrameDistance = 25;
    public double maxFrameDistance = 700;

    double carriageTolerance = 20.0;
    double frameTolerance = 40.0; 
    double carriagePID = 0.003;
    double framePID = 0.005;

    private final double IDLE_DUTY_CYCLE = 0.02;

    private PIDController pidController = new PIDController(0.005, 0, 0);
    
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
        rightMotorConfig.smartCurrentLimit(38);
        motorRight.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Left Elevator Motor (Follows Right Elevator Motor)
        motorLeft = new SparkMax(Constants.Elevator.LEFT_ELEVATOR_MOTOR, MotorType.kBrushless);
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.idleMode(IdleMode.kCoast);
        leftMotorConfig.follow(motorRight, true);
        leftMotorConfig.smartCurrentLimit(38);
        motorLeft.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        positions.put(ElevatorState.POLE, new Double[]{maxFrameDistance, minCarriageDistance});
        positions.put(ElevatorState.SAFE, new Double[]{minFrameDistance, minCarriageDistance});
        positions.put(ElevatorState.SAFE_CORAL, new Double[]{minFrameDistance, 120.0});
        // positions.put(ElevatorState.L1, new Double[]{0.0, 0.0});
        positions.put(ElevatorState.L2, new Double[]{minFrameDistance, 540.0});
        positions.put(ElevatorState.L3, new Double[]{minFrameDistance, minCarriageDistance});
        positions.put(ElevatorState.L3_LOW, new Double[]{minFrameDistance, 350.0});
        positions.put(ElevatorState.L4, new Double[]{maxFrameDistance, minCarriageDistance});
        positions.put(ElevatorState.BARGE_SCORE, new Double[]{maxFrameDistance, minCarriageDistance});
        positions.put(ElevatorState.COLLECT_LOW, new Double[]{minFrameDistance, 257.0});
        positions.put(ElevatorState.GROUND_COLLECT, new Double[]{minFrameDistance, 290.0});
        positions.put(ElevatorState.HIGH_ALGAE, new Double[]{minFrameDistance, minCarriageDistance});
        positions.put(ElevatorState.LOW_ALGAE, new Double[]{maxFrameDistance, 350.0});
        positions.put(ElevatorState.SMART_ALGAE, new Double[]{minFrameDistance, minCarriageDistance});
    }
    
    public Command idle(BooleanSupplier isArmInsideElevator, BooleanSupplier clawCoralPresent) {
        return new InstantCommand(() -> {
            Logger.recordOutput("Elevator/CurrentCommand", "IDLE");
            if(isArmInsideElevator.getAsBoolean()) {
                Logger.recordOutput("Elevator/CurrentCommand", "Waiting for arm to clear (IDLE)");
                elevatorDutyCycle = IDLE_DUTY_CYCLE;
            } else if (clawCoralPresent.getAsBoolean() && !hasReachedPosition(ElevatorState.L4)) {
                new SetElevator(this, ElevatorState.L3).schedule();
                Logger.recordOutput("Elevator/CurrentCommand", "moving to L3 for coral");
            } else if (!clawCoralPresent.getAsBoolean()) {
                new SetElevator(this, ElevatorState.SAFE_CORAL).schedule();
                Logger.recordOutput("Elevator/CurrentCommand", "moving to Precollect for collecting");
            } else {
                elevatorDutyCycle = IDLE_DUTY_CYCLE;
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
        Logger.recordOutput("Elevator/hasReachedSafePosition", hasReachedPosition(ElevatorState.SAFE));
        Logger.recordOutput("Elevator/current", motorRight.getOutputCurrent());
        Logger.recordOutput("Elevator/trueDC", motorRight.getAppliedOutput());
        if (getFramePosition() > maxFrameDistance || getCarriagePosition() > maxCarriageDistance) {
            elevatorDutyCycle = IDLE_DUTY_CYCLE;
        }

        motorRight.set(clampDutyCycle(elevatorDutyCycle));
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
        double tolerance = 70;
        return firstStageDiff <= tolerance && secondStageDiff <= tolerance;
    }

    public boolean isFinishedSettingElevator(ElevatorState desiredState) {
        if(desiredState == ElevatorState.STOP) {
            return true;
        }
        double carriagePosition = getCarriagePosition();
        double framePosition = getFramePosition();
        double desiredCarriagePosition = positions.get(desiredState)[1];
        double desiredFramePosition = positions.get(desiredState)[0];
        double frameDiff = Math.abs(desiredFramePosition - framePosition);
        double carriageDiff = Math.abs(desiredCarriagePosition - carriagePosition);
        return (frameDiff <= frameTolerance) && 
        (carriageDiff <= carriageTolerance || desiredFramePosition > minFrameDistance + 20); //advanced logic
    }

    public void setElevatorPosition(ElevatorState desiredState) {

        if(desiredState == ElevatorState.STOP || desiredState == ElevatorState.MANUAL_DOWN || desiredState == ElevatorState.MANUAL_UP) {
            elevatorDutyCycle = IDLE_DUTY_CYCLE;
            return;
        }

        if (positions.get(desiredState)[0] > minFrameDistance && !robotContainer.getSensation().clawCoralPresent()) {
            System.out.println("Elevator should not reach state " + desiredState + " because coral is not present in claw");
            System.out.println(positions.get(desiredState)[0]);
            System.out.println(robotContainer.getSensation().clawCoralPresent());
            return;
        }

        isIdle = false;
        double carriagePosition = getCarriagePosition();
        double framePosition = getFramePosition();
        

        Double[] desiredPosition = positions.get(desiredState);
         if (frameTOF.isRangeValidRegularCheck() && Math.abs(desiredPosition[0] - framePosition) >= frameTolerance) {
            // double calculatedDutyCycle = pidController.calculate(desiredPosition[0] - framePosition);
            double calculatedDutyCycle = carriagePID * (desiredPosition[0] - framePosition);
            double clampedResult = clampDutyCycle(calculatedDutyCycle);
            elevatorDutyCycle = clampedResult;
        } 
        else if (desiredPosition[0] < minFrameDistance + 20 && carriageTOF.isRangeValidRegularCheck() && Math.abs(desiredPosition[1] - carriagePosition) >= carriageTolerance) {
            // double calculatedDutyCycle = pidController.calculate(desiredPosition[1] - carriagePosition);
            double calculatedDutyCycle = -framePID * (desiredPosition[1] - carriagePosition);
            double clampedResult = clampDutyCycle(calculatedDutyCycle);
            elevatorDutyCycle = clampedResult;
        } else {
            pidController.reset();
            elevatorDutyCycle = IDLE_DUTY_CYCLE;
        }

        // if (((!robotContainer.isSafeForElevatorCarriagetoMove()) && Math.abs(carriagePosition - desiredPosition[1]) > 100)) { //|| !robotController.driveController.isSafeForElevatorStage2toMove()) && Math.abs(secondStagePosition - desiredPosition[1]) > 100) {
        //     elevatorDutyCycle = IDLE_DUTY_CYCLE;
        // }
    }

    public Command moveUp() {
        return new InstantCommand(() -> {
            setElevatorPosition(ElevatorState.MANUAL_UP);
            elevatorDutyCycle = 0.35;
        }, this);
    }

    public Command moveDown() {
        return new InstantCommand(() -> {
            setElevatorPosition(ElevatorState.MANUAL_DOWN);
            elevatorDutyCycle = -0.35;
        }, this);
    }

    public double clampDutyCycle(double dutyCycle) {
        return EEUtil.clamp(-0.75, 0.75, dutyCycle);
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
