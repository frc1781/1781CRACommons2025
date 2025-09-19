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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.EEtimeOfFlight;

public class Arm extends SubsystemBase {
    private double targetPosition;
    private SparkMax armMotor;
    private ArmState currentState;
    private SparkMaxConfig armMotorConfig;

    // No Coral Cycle 0.095 @ 12 volts
    // With Coral 0.105
    // With Algae 0.15
    // Slot 0 P is 0.004 was 0.008

    public Arm() {
        armMotor = new SparkMax(Constants.Arm.ARM_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        armMotor.setControlFramePeriodMs(20);
        armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        armMotorConfig.smartCurrentLimit(40);
        armMotorConfig.absoluteEncoder.positionConversionFactor(360);
        armMotorConfig.absoluteEncoder.zeroOffset(0.4868528);
        armMotorConfig.closedLoop.pid(0.004, 0,0.001);
        armMotorConfig.closedLoop.velocityFF((double) 1 /565); // https://docs.revrobotics.com/brushless/neo/vortex#motor-specifications
        armMotorConfig.closedLoop.outputRange(-.55, .55); //(-.55, .55);
        armMotorConfig.closedLoop.positionWrappingEnabled(true);
        armMotorConfig.softLimit.forwardSoftLimit(180);
        armMotorConfig.softLimit.reverseSoftLimit(0);
        armMotorConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        currentState = ArmState.IDLE;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Arm/DutyCycle", armMotor.getAppliedOutput());
        Logger.recordOutput("Arm/currentPosition", getPosition());
        Logger.recordOutput("Arm/targetPosition", targetPosition);
        Logger.recordOutput("Arm/currentState", currentState.toString());
        
        if (currentState == ArmState.MANUAL_UP) {
            targetPosition = getPosition() - 0.2;
            currentState = ArmState.IDLE;
        }
        else if (currentState == ArmState.MANUAL_DOWN) {
            targetPosition = getPosition() + 0.2;
            currentState = ArmState.IDLE;
        }
        else if (!RobotContainer.isSafeForArmToMove() || currentState == ArmState.IDLE) {
            targetPosition = getPosition(); //will not move unless manually controlled
        }
        else {
            targetPosition = currentState.getPosition();
        }

        double gravityFeedForward = -0.095 * Math.sin(Rotation2d.fromDegrees(getPosition()).getRadians());

        // armMotor.getClosedLoopController().setReference(
        //     targetPosition,
        //     ControlType.kPosition,
        //     ClosedLoopSlot.kSlot0,
        //     gravityFeedForward,
        //     SparkClosedLoopController.ArbFFUnits.kPercentOut
        // );     
    }

    public void setState(ArmState newState) {
        currentState = newState;
    }

    public double getPosition() {
        return armMotor.getAbsoluteEncoder().getPosition();
    }

    public boolean matchesState() {
        if (currentState == ArmState.MANUAL_DOWN || currentState == ArmState.MANUAL_UP) {
            return true;
        }

        return Math.abs(targetPosition - getPosition()) <= 8; //tollorence
    }

    public enum ArmState  {
        IDLE(Double.NaN),
        MANUAL_UP(Double.NaN),      // No fixed position, or use a special value
        MANUAL_DOWN(Double.NaN),
        L1(45.0),
        L2(0.0),
        L3(28.0),
        L4(65.0),
        COLLECT(179.0),
        WAIT(25.0),
        POLE(25.0),
        START_MID(40.0),
        START_HIGH(5.0),
        GROUND_ALGAE(159.0),
        REEF_ALGAE(60.0),
        READY_ALGAE(25.0),
        SLIGHT_TOSS(21.0);
    
        private final double position;
    
        ArmState(double position) {
            this.position = position;
        }
    
        public double getPosition() {
            return position;
        }
    }
}
