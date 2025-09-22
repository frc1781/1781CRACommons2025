// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Clear;
import frc.robot.commands.Collect;
import frc.robot.commands.L4;
import frc.robot.commands.PostCollect;
import frc.robot.commands.PreCollect;
import frc.robot.commands.SafeConfig;
import frc.robot.commands.Score;
import frc.robot.commands.SetElevator;
import frc.robot.commands.Shoot;
import frc.robot.commands.StrafeCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.NoSuchElementException;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveInputStream;

public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  private Sensation sensation = new Sensation();;
  private SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/ava"));; //im sure this wont cause issues later
  // private final TankDriveTrain tankDrive = new TankDriveTrain(driverXbox);
  private final Conveyor conveyor = new Conveyor();
  private final Lights lights = new Lights();
  private final Elevator elevator = new Elevator(this);
  private final Arm arm = new Arm(this);
  // private final Climber climber = new Climber();
  private final SendableChooser<Command> autoChooser;
  private double wait_seconds = 5;

   Trigger coralEnter = new Trigger(sensation::coralPresent);
   Trigger coralHopper = new Trigger(sensation::coralInHopper);
   Trigger coralExit = new Trigger(sensation::coralExitedHopper);
   Trigger robotInPosition = new Trigger(this::inPosition);
   Trigger readyToCollectTrigger = new Trigger(this::readyToCollect);

  //Driving the robot during teleOp
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
    () -> driverXbox.getLeftY() * -1,
    () -> driverXbox.getLeftX() * -1)
    .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)  
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)  //might be changed to 1
    .allianceRelativeControl(true)
    .cubeRotationControllerAxis(true);

  //Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
    .withControllerHeadingAxis(() -> driverXbox.getRightX() * -1, () -> driverXbox.getRightY() * -1)
    .headingWhile(true);

   // Clone's the angular velocity input stream and converts it to a robotRelative input stream.
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
    .robotRelative(true)
    .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
    () -> -driverXbox.getLeftY(),
    () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis( 2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
    .withControllerHeadingAxis(
      () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) *(Math.PI *2),
      () -> Math.cos(driverXbox.getRawAxis(2) *Math.PI) *(Math.PI *2))
        .headingWhile(true)
        .translationHeadingOffset(true)
        .translationHeadingOffset(Rotation2d.fromDegrees( 0));

  private boolean isManualControlMode;

  public RobotContainer()
  {
    NamedCommands.registerCommand("CustomWaitCommand", new WaitCommand(SmartDashboard.getNumber("Wait Time", wait_seconds)));
    NamedCommands.registerCommand("Score", new Score(arm));
    NamedCommands.registerCommand("Collect", new Collect(elevator, coralEnter));
    NamedCommands.registerCommand("MoveToPositionToScore", drivebase.new MoveToPositionToScore(sensation));
    NamedCommands.registerCommand("Clear", new Clear(arm));
    NamedCommands.registerCommand("StrafeCommand", new StrafeCommand(drivebase, elevator, arm, sensation, true));
    NamedCommands.registerCommand("L4", new L4(elevator, arm));
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putNumber("Wait Time", wait_seconds);
  }

  public void periodic() {
    Logger.recordOutput("RobotContainer/isSafeForElevatortoMoveUp", isSafeForElevatorCarriagetoMove());
    Logger.recordOutput("RobotContainer/isSafeForArmToMoveUp", isSafeForArmToMoveUp());
    Logger.recordOutput("RobotContainer/isSafeForArmToMoveDown", isSafeForArmToMoveDown());
    Logger.recordOutput("RobotContainer/isArmInsideElevator", isArmInsideElevator());
  }

  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    //Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    //Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    //Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    //Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    //Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } 
    else
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    }

    conveyor.setDefaultCommand(conveyor.clearCoral(coralHopper));
    lights.setDefaultCommand(lights.set(Lights.Special.OFF));
    elevator.setDefaultCommand(elevator.idle());
    sensation.setDefaultCommand(Commands.idle(sensation));
    arm.setDefaultCommand(arm.idle());
   // climber.setDefaultCommand(Commands.);

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      driveDirectAngleKeyboard.driveToPose(
        () -> target, 
        new ProfiledPIDController(5, 0,0, new Constraints(5, 2)),
        new ProfiledPIDController(5,0,0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                      () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
    }

    if (DriverStation.isTest())
    {
      //drivebase.setDefaultCommand(driveFieldOrienteAnglularVelocity); // Overrides drive command above!d
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } 
    else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.povLeft().whileTrue(arm.manualUp().repeatedly());
      // driverXbox.povRight().whileTrue(arm.manualDown().repeatedly());      
      // driverXbox.b().whileTrue(Commands.run(() -> arm.setState(ArmState.L1)));
      driverXbox.x().onTrue(new SafeConfig(elevator, arm));
      driverXbox.povUp().onTrue(new PreCollect(elevator, arm));
      driverXbox.povRight().onTrue(new PostCollect(elevator, arm));
      driverXbox.povDown().onTrue(new L4(elevator, arm));
      driverXbox.povLeft().onTrue(new Score(arm));
      //driverXbox.x().whileTrue(Commands.run(() -> arm.setState(ArmState.REEF_ALGAE)));
      //driverXbox.x().whileTrue(Commands.run(() -> elevator.setState(Elevator.ElevatorState.L4)));
      driverXbox.rightBumper().onTrue(Commands.none());
     // driverXbox.povUp().whileTrue(climber.ascend());
     // driverXbox.povDown().whileTrue(climber.descend());
     //driverXbox.y().onTrue(lights.set(Lights.Special.RAINBOW));
     
     driverXbox.y().whileTrue(drivebase.new MoveToPositionToScore(sensation)
        .andThen(new StrafeCommand(drivebase, elevator, arm, sensation, true))
     );
     
     //driverXbox.b().onTrue(lights.set(Lights.Colors.WHITE, Lights.Patterns.MARCH));
     
     //TRIGGERS

     robotInPosition.whileTrue(lights.set(Lights.Colors.GREEN, Lights.Patterns.SOLID));
     coralEnter.and(coralExit.negate()).and(coralHopper.negate()).onTrue(lights.set(Lights.Colors.RED, Lights.Patterns.FAST_FLASH));
     coralHopper.and(coralExit.negate()).onTrue(lights.set(Lights.Colors.RED, Lights.Patterns.MARCH));
     coralExit.onFalse(lights.set(Lights.Colors.RED, Lights.Patterns.SOLID));
     coralExit.and(readyToCollectTrigger).onTrue(new Collect(elevator, sensation::clawCoralPresent));
    }
  }

  public static boolean isRed() {
        try {
            return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        }
        catch (NoSuchElementException e) {
            return false;
        }
  }

  public boolean isSafeForElevatorCarriagetoMove() {
    return arm.getPosition() > 40.0 && arm.getPosition() < 300;  //should never be this high except with gimble lock wrapping
  }

  public boolean isSafeForArmToMoveUp() {
    double maxUnsafeDistance = 100;  //THESE ARE JUST PLACEHOLDER VALUES CHANGE THEM ASAP
    double minUnsafeDistance = 200;  //THESE ARE JUST PLACEHOLDER VALUES CHANGE THEM ASAP
    return elevator.getCarriagePosition() < minUnsafeDistance || elevator.getCarriagePosition() > maxUnsafeDistance;
  }

  public boolean isSafeForArmToMoveDown() {
    double maxUnsafeDistance = elevator.maxCarriageDistance;
    return elevator.getCarriagePosition() > maxUnsafeDistance;
  }

  public boolean isArmInsideElevator() {
    return elevator.getCarriagePosition() < 50 && arm.getPosition() < 30;
  }

  public boolean readyToCollect() {
    return elevator.hasReachedPosition(ElevatorState.SAFE) && arm.matchesState(ArmState.COLLECT);
  }

  public boolean isManualControlMode() {
    return isManualControlMode();
  }

  public void teleopInit() {
    drivebase.setMotorBrake(true);
    arm.setState(ArmState.START);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand()
    {
      return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake)
    {
      drivebase.setMotorBrake(brake);
    }

    public boolean inPosition() {
      return 
         sensation.leftTOFisValid() && 
         sensation.rightTOFisValid() && 
         sensation.leftTOF() < 1000 && 
         sensation.rightTOF() < 1000;
         //robotController.visionSystem.getDoubleCameraReefApriltag() != -1;
    }

  }
