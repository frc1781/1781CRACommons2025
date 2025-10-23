package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Clear;
import frc.robot.commands.L3;
import frc.robot.commands.PreCollect;
import frc.robot.commands.SetArm;
import frc.robot.utils.EEUtil;
import frc.robot.utils.EEtimeOfFlight;

public class Arm extends SubsystemBase {
    private double targetPosition;
    private double currentPosition;
    private SparkMax armMotor;
    private ArmState currentState = ArmState.START;
    private SparkFlex spinMotor;
    private ThumbState currentThumbState = ThumbState.IDLE;
    private double dutyCycle = 0;

    private SparkMaxConfig armMotorConfig;
    private RobotContainer robotContainer;


    // No Coral Cycle 0.095 @ 12 volts
    // With Coral 0.105
    // With Algae 0.15
    // Slot 0 P is 0.004 was 0.008

    public Arm(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        currentState = ArmState.START;
        currentThumbState = ThumbState.IDLE;
        armMotor = new SparkMax(Constants.Arm.ARM_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        armMotor.setControlFramePeriodMs(20);
        armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        armMotorConfig.smartCurrentLimit(40);
        armMotorConfig.absoluteEncoder.positionConversionFactor(360);
        armMotorConfig.absoluteEncoder.zeroOffset(0.315);
        armMotorConfig.closedLoop.pid(0.004, 0,0.000);
        armMotorConfig.closedLoop.velocityFF((double) 1 /565); // https://docs.revrobotics.com/brushless/neo/vortex#motor-specifications
        armMotorConfig.closedLoop.outputRange(-.25, .25); //(-.55, .55);
        armMotorConfig.closedLoop.positionWrappingEnabled(true);
        armMotorConfig.softLimit.forwardSoftLimit(180);
        armMotorConfig.softLimit.reverseSoftLimit(0);
        armMotorConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        spinMotor = new SparkFlex(Constants.Arm.THUMB_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        SparkFlexConfig spinMotorConfig = new SparkFlexConfig();
        spinMotorConfig.idleMode(IdleMode.kBrake);
        spinMotorConfig.smartCurrentLimit(30);
        spinMotorConfig.inverted(false);
        spinMotor.configure(spinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command idle(BooleanSupplier isSafeForArmToMoveUp, BooleanSupplier isSafeForArmToMoveDown, BooleanSupplier clawCoralPresent) {
        //Arm should always be in L3 when doing nothing
        return new InstantCommand(() -> {
                //the only time it is not safe to move up is when the arm is in collect with a coral and the elevator is down and just collected, needs to move up a bit first
            if (isSafeForArmToMoveUp.getAsBoolean() || getPosition() < ArmState.L3.getPosition()) {
                if (!clawCoralPresent.getAsBoolean()) {
                    new SetArm(this, ArmState.COLLECT).schedule();
                } else {
                    new SetArm(this, ArmState.START_MID).schedule();
                }
            }
        }, this);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Arm/DutyCycle", armMotor.getAppliedOutput());
        Logger.recordOutput("Arm/currentPosition", getPosition());
        Logger.recordOutput("Arm/targetPosition", targetPosition);
        Logger.recordOutput("Arm/currentState", currentState.toString());
        Logger.recordOutput("Arm/matchesState", matchesState());
        
        if (currentState == ArmState.MANUAL_UP) {
            targetPosition += -0.2;
            System.out.println("decrementing target " + targetPosition);
        }

        if (currentState == ArmState.MANUAL_DOWN) {
            targetPosition += 0.2;
            System.out.println("incrementing target " + targetPosition);
        }
        
        double gravityFeedForward = -0.095  * Math.sin(Rotation2d.fromDegrees(getPosition()).getRadians());
        Logger.recordOutput("Arm/FFValue", gravityFeedForward);
        //if (/*RobotContainer.isSafeForArmToMoveUp() ||*/ currentState == ArmState.MANUAL_UP || currentState == ArmState.MANUAL_DOWN){
           // target = targetPosition;
        //}
        
        if (currentState == ArmState.START) {
            return;
        }

        if (currentState == ArmState.STOP) {
            armMotor.set(0.0);
            return;
        }

        //keep arm from moving up if elevator is too low
        if (armNeedsToMoveUp() && !robotContainer.isSafeForArmToMoveUp()) {
            armMotor.set(0.0);
            return;
        }

        //thumb stuff
        if (currentState == ArmState.REEF_ALGAE || currentState == ArmState.GROUND_ALGAE) {
            currentThumbState = ThumbState.SPIN_IN;
        } else if (currentState == ArmState.READY_ALGAE || currentState == ArmState.SLIGHT_TOSS) {
            currentThumbState = ThumbState.SPIN_OUT;
        } else {
            currentThumbState = ThumbState.IDLE;            
        }

        spinMotor.set(currentThumbState.getDutyCycle());
        
        // armMotor.getClosedLoopController().setReference(
        //     targetPosition,
        //     ControlType.kPosition,
        //     ClosedLoopSlot.kSlot0,
        //     gravityFeedForward,
        //     SparkClosedLoopController.ArbFFUnits.kPercentOut
        // );  

        //manual PID!!
        dutyCycle = 0.004*(targetPosition - getPosition()) + gravityFeedForward;   //Basic P control plus gravity feedforward for PID
        if (Math.abs(armMotor.getAppliedOutput() - dutyCycle) > 0.3) {
            dutyCycle = armMotor.getAppliedOutput() + Math.signum(dutyCycle - armMotor.getAppliedOutput()) * 0.03;
        }  //rate limit changes via actual applied output
        armMotor.set(clampDutyCycle(dutyCycle));  //clamp and set duty cycle
    }

    private boolean armNeedsToMoveUp() {
        return getPosition() > targetPosition;
    }

    public void setState(ArmState newState) {
        if (currentState == newState) {
            return; //do nothing
        }

        if (newState == ArmState.STOP) {
            currentState = ArmState.STOP;
            return;
        }

        //don't go idle unless manual or starting
        if (newState == ArmState.IDLE && currentState != ArmState.MANUAL_UP && currentState != ArmState.MANUAL_DOWN && currentState != ArmState.START) {
            return;
        }

        currentState = newState;
        currentPosition = getPosition();
        if (newState == ArmState.IDLE  || newState == ArmState.MANUAL_UP || newState == ArmState.MANUAL_DOWN) {
            targetPosition = currentPosition;
            System.out.println("get target from currentPosition: " + targetPosition);
        }
        else {
            targetPosition = newState.getPosition();  
            System.out.println("get target from table: " + targetPosition);
        }
    }
    
    public Command manualDown(){
        return new InstantCommand(() -> {
            setState(ArmState.MANUAL_DOWN);
        }, this);
    }
    
    public Command manualUp(){
        return new InstantCommand(() -> {
            setState(ArmState.MANUAL_UP);
        }, this);
    }

    public double getPosition() {
        return armMotor.getAbsoluteEncoder().getPosition();
    }

    public boolean matchesState() {
        if (currentState == ArmState.MANUAL_DOWN || currentState == ArmState.MANUAL_UP) {
            return true;
        }

        return Math.abs(targetPosition - getPosition()) <= 8; //tolerance
    }

    public boolean matchesState(ArmState state) {
        return Math.abs(state.getPosition() - getPosition()) <= 8.0; //tolerance
    }

    public double clampDutyCycle(double dutyCycle) {
        return EEUtil.clamp(-0.35, 0.35, dutyCycle);
    }

    public enum ArmState  {
        START(Double.NaN),
        IDLE(Double.NaN),
        MANUAL_UP(Double.NaN),      // No fixed position, or use a special value
        MANUAL_DOWN(Double.NaN),
        STOP(Double.NaN),
        L1(45.0),
        L2(90.0), // why was this 0.0
        L3(25.0),
        SCORE_L4(75.0),
        COLLECT(179.5),
        WAIT(25.0),
        POLE(28.0),
        SCORE_MID(60.0),
        START_MID(35.0),
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

    public enum ThumbState {
        SPIN_IN(-1), SPIN_OUT(1), IDLE(0);

        private final double dutyCycle;

        ThumbState(double dutyCycle) {
            this.dutyCycle = dutyCycle;
        }

        public double getDutyCycle() {
            return dutyCycle;
        }
    }
}
