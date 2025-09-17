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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.EEUtil;
import frc.robot.utils.EEtimeOfFlight;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase{


    private boolean isIdle = true;

    private ElevatorState currentState;

    private final SparkMax motorRight;

    private SparkMax motorLeft;

    private EEtimeOfFlight firstStageTOF;
    private EEtimeOfFlight secondStageTOF;
    // measure the max distance
    private double minSecondStageDistance = 0;
    private double maxSecondStageDistance = 680;

    private double minFirstStageDistance = 0;
    private double maxFirstStageDistance = 810; 

    private ElevatorState previousSmartState;

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
        firstStageTOF = new EEtimeOfFlight(Constants.Elevator.FIRST_STAGE_TOF, 20);
        secondStageTOF = new EEtimeOfFlight(Constants.Elevator.SECOND_STAGE_TOF, 20);

        //Right Elevator Motor
        motorRight = new SparkMax(Constants.Elevator.RIGHT_ELEVATOR_MOTOR, MotorType.kBrushless);

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.idleMode(IdleMode.kCoast);
        rightMotorConfig.smartCurrentLimit(30);
        motorRight.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Left Elevator Motor
        motorLeft = new SparkMax(Constants.Elevator.LEFT_ELEVATOR_MOTOR, MotorType.kBrushless);
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.idleMode(IdleMode.kCoast);
        leftMotorConfig.follow(motorRight, true);
        leftMotorConfig.smartCurrentLimit(30);
        motorLeft.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        positions.put(ElevatorState.POLE, new Double[]{750.0, minSecondStageDistance});
        positions.put(ElevatorState.SAFE, new Double[]{minFirstStageDistance, 80.0});
        positions.put(ElevatorState.SAFER, new Double[]{minFirstStageDistance, 80.0});
        positions.put(ElevatorState.L1, new Double[]{0.0, 0.0});
        positions.put(ElevatorState.L2, new Double[]{minFirstStageDistance, 80.0});
        positions.put(ElevatorState.L3, new Double[]{minFirstStageDistance, 165.0});
        positions.put(ElevatorState.L3_LOW, new Double[]{minFirstStageDistance, 350.0});
        positions.put(ElevatorState.L4, new Double[]{maxFirstStageDistance, minSecondStageDistance});
        positions.put(ElevatorState.BARGE_SCORE, new Double[]{maxFirstStageDistance, minSecondStageDistance});
        positions.put(ElevatorState.COLLECT_LOW, new Double[]{minFirstStageDistance, 400.0});
        positions.put(ElevatorState.GROUND_COLLECT, new Double[]{0.0, 290.0});
        positions.put(ElevatorState.HIGH_ALGAE, new Double[]{minFirstStageDistance, minSecondStageDistance});
        positions.put(ElevatorState.LOW_ALGAE, new Double[]{maxFirstStageDistance, 350.0});
        positions.put(ElevatorState.SMART_ALGAE, new Double[]{minFirstStageDistance, 50.0});
    }

    public Command idle() {
        return new InstantCommand(() -> {
            isIdle = true;
        }, this);
    }

    public double getFirstStagePosition() {
        return firstStageTOF.getRange();
    }

    public double getSecondStagePosition() {
        return secondStageTOF.getRange();
    }

    // public ElevatorState getSmartAlgaeState() {
    //     int apriltag = RobotContainer.visionSystem.getDoubleCameraReefApriltag();        //NEED TO INCORPORATE VISION

    //     if (apriltag == -1 && previousSmartState != null) {
    //         return previousSmartState;
    //     }

    //     if (apriltag == -1 && previousSmartState == null) {
    //         return ElevatorState.LOW_ALGAE;
    //     }

    //     if (RobotContainer.isRed()) {
    //         if (apriltag % 2 != 0) {
    //             return ElevatorState.LOW_ALGAE;
    //         } else {
    //             return ElevatorState.HIGH_ALGAE;
    //         }
    //     } else {
    //         if (apriltag % 2 == 0) {
    //             return ElevatorState.LOW_ALGAE;
    //         } else {
    //             return ElevatorState.HIGH_ALGAE;
    //         }
    //     }
    // }

    public void goToPosition() {
        double firstStagePosition = getFirstStagePosition();
        double secondStagePosition = getSecondStagePosition();
        double dutyCycle = 0;
        Double[] desiredPosition = positions.get(getCurrentState());
        // if (getCurrentState() == ElevatorState.SMART_ALGAE) {
        //     ElevatorState smartAlgaeState = getSmartAlgaeState();
        //     if (previousSmartState == null || previousSmartState != smartAlgaeState) {
        //         previousSmartState = smartAlgaeState;
        //     }

        //     Logger.recordOutput("Elevator/smartAlgaeState", smartAlgaeState);
        //     desiredPosition = positions.get(smartAlgaeState);
        // }
        double Tolerance = 80;
        
        if (secondStageTOF.isRangeValidRegularCheck() && Math.abs(desiredPosition[1] - secondStagePosition) >= Tolerance) {
            double ff = -feedforwardController.calculate(desiredPosition[1] - secondStagePosition);
            Logger.recordOutput("Elevator/FFUnClamped", ff);
            double clampedResult = clampDutyCycle(ff);
            Logger.recordOutput("Elevator/FFClampedOutput", clampedResult);
            dutyCycle = clampedResult;
        } else if (firstStageTOF.isRangeValidRegularCheck() && Math.abs(desiredPosition[0] - firstStagePosition) > Tolerance) {
            double ff = feedforwardController.calculate(desiredPosition[0] - firstStagePosition);
            Logger.recordOutput("Elevator/FFUnClamped", ff);
            double clampedResult = clampDutyCycle(ff);
            Logger.recordOutput("Elevator/FFClampedOutput", clampedResult);
            dutyCycle = clampedResult;
        } else {
            dutyCycle = 0.02;
        }

        //--------------------------------------------------------------
        //The second stage of the elevator (the inner stage) actually moves up first.
        //Well, the arm moves up until it hits the top of the 2nd stage then that stage starts going up.
        //If the arm is at the top of the second stage the first stage can move.  If it is not at the top of the
        //second stage then the second stage should not move unless it is out of the way of hitting the top.
        //It really only starts low at the beginning when it is not safe to move the second stage until the arm is moved
        //out.  But once it is up at the top of the second stage it can move into positions that make it dangerous
        //to leave it's spot on the second stage until it is back in a safe position.
        if ((/*(!RobotContainer.isSafeForElevatorStage2toMove()) && */ Math.abs(secondStagePosition - desiredPosition[1]) > 100)) { //|| !robotController.driveController.isSafeForElevatorStage2toMove()) && Math.abs(secondStagePosition - desiredPosition[1]) > 100) {
            dutyCycle = 0.02;
        }

        Logger.recordOutput("Elevator/DutyCycle", dutyCycle);
        motorRight.set(dutyCycle);
    }


    public ElevatorState getCurrentState() {
        return currentState;
    }

    public double clampDutyCycle(double dutyCycle) {
        if (getCurrentState() == ElevatorState.COLLECT_LOW || getCurrentState() == ElevatorState.L3 || getCurrentState() == ElevatorState.L2) {
            return EEUtil.clamp(0.0, 0.75, dutyCycle);
        }

        return EEUtil.clamp(0, 0.8, dutyCycle);
    }

   

    public enum ElevatorState {
        SAFE,
        SAFER,
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
