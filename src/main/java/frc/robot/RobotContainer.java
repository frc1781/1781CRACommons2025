// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TargetSide;
import frc.robot.commands.CenterAndScore;
import frc.robot.commands.Clear;
import frc.robot.commands.Collect;
import frc.robot.commands.CollectAndClear;
import frc.robot.commands.CollectAndPost;
import frc.robot.commands.Collecting;
import frc.robot.commands.L2;
import frc.robot.commands.L3;
import frc.robot.commands.L4;
import frc.robot.commands.MoveBack;
import frc.robot.commands.MoveToTarget;
import frc.robot.commands.PostCollect;
import frc.robot.commands.PreCollect;
import frc.robot.commands.SafeConfig;
import frc.robot.commands.ScoreL4;
import frc.robot.commands.ScoreLow;
import frc.robot.commands.SetArm;
import frc.robot.commands.SetElevator;
import frc.robot.commands.SetTargetPose;
//import frc.robot.commands.StopMovingToTarget;
import frc.robot.commands.StrafeCommand;
import frc.robot.commands.WaitForCoral;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import swervelib.SwerveInputStream;

public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController copilotXbox = new CommandXboxController(1);
  final CommandJoystick copilotButtons = new CommandJoystick(2);
  private Sensation sensation = new Sensation();
  private SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/ava"));
  private final Conveyor conveyor = new Conveyor();
  private final Lights lights = new Lights();
  private final Elevator elevator = new Elevator(this);
  private final Arm arm = new Arm(this);
  private final Climber climber = new Climber();
  private final SendableChooser<Command> autoChooser;
  private double wait_seconds = 5;
  public int targetAprilTagID = -1;
  public TargetSide targetedSide = TargetSide.LEFT;
  

  Trigger coralPresent = new Trigger(sensation::coralPresent);
  Trigger coralHopper = new Trigger(sensation::coralInHopper);
  Trigger coralExit = new Trigger(sensation::coralExitedHopper);
  Trigger armHasCoral = new Trigger(sensation::clawCoralPresent);
  Trigger robotInPosition = new Trigger(this::inPosition);
  Trigger readyToCollectTrigger = new Trigger(this::readyToCollect);
  Trigger isTeleopTrigger = new Trigger(DriverStation::isTeleop);
  Trigger isRedAllianceTrigger = new Trigger(RobotContainer::isRed);
  Trigger coralInClaw = new Trigger(sensation::clawCoralPresent);

  // Driving the robot during teleOp
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8) // might be changed to 1
      .allianceRelativeControl(true)
      .cubeRotationControllerAxis(true);

  // Clone's the angular velocity input stream and converts it to a fieldRelative
  // input stream.
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(() -> driverXbox.getRightX() * -1, () -> driverXbox.getRightY() * -1)
      .headingWhile(true);

  // Clone's the angular velocity input stream and converts it to a robotRelative
  // input stream.
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
      .robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(
          () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
          () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(0));

  private boolean isManualControlMode;

  public RobotContainer() {
    SmartDashboard.putNumber("Target Apriltag", targetAprilTagID);
    NamedCommands.registerCommand("Score", new ScoreL4(arm, drivebase));
    NamedCommands.registerCommand("Collect", new CollectAndClear(elevator, arm, sensation));
    NamedCommands.registerCommand("MoveToPositionToScore", drivebase.new MoveToPositionToScore(sensation));
    NamedCommands.registerCommand("Clear", new Clear(arm));
    NamedCommands.registerCommand("StrafeCommand", new StrafeCommand(drivebase, elevator, arm, sensation, true));
    NamedCommands.registerCommand("L4", new L4(elevator, arm));
    NamedCommands.registerCommand("PreCollect", new PreCollect(elevator, arm, sensation));
    NamedCommands.registerCommand("PostCollect", new PostCollect(elevator, arm));
    NamedCommands.registerCommand("SetElevator", new SetElevator(elevator, ElevatorState.SAFE));
    NamedCommands.registerCommand("SetArm", new SetArm(arm, ArmState.START));
    NamedCommands.registerCommand("SafeConfig", new SafeConfig(elevator, arm));
    NamedCommands.registerCommand("MoveBack", new MoveBack(drivebase));
    NamedCommands.registerCommand("WaitForCoral", new WaitForCoral(sensation, drivebase));

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("CustomWaitCommand", new WaitCommand(SmartDashboard.getNumber("Wait Time", wait_seconds)));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      Logger.recordOutput("Drive/currentPose", pose);
    });

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      Logger.recordOutput("Drive/targetPose", pose);
    });

    PathPlannerLogging.setLogActivePathCallback((path) -> {
      if (path != null) {
        Logger.recordOutput("Drive/CurrentCommand/", "RunningPath");
      }
    });
  }

  public void periodic() {
    SmartDashboard.getNumber("Target Apriltag", targetAprilTagID);
    Logger.recordOutput("RobotContainer/isSafeForArmToMoveUp", isSafeForArmToMoveUp());
    Logger.recordOutput("RobotContainer/isSafeForArmToMoveDown", isSafeForArmToMoveDown());
    Logger.recordOutput("RobotContainer/isArmInsideElevator", isArmInsideElevator());
    Logger.recordOutput("RobotContainer/readyToCollect", readyToCollect());
    Logger.recordOutput("RobotContainer/targetAprilTagID", targetAprilTagID);
    Logger.recordOutput("RobotContainer/targetPose", scorePose(targetAprilTagID, targetedSide));
    Logger.recordOutput("targetedSide", targetedSide.toString());
  }

  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    // Command driveFieldOrientedAnglularVelocity =
    // drivebase.driveFieldOriented(driveAngularVelocity);
    // Command driveRobotOrientedAngularVelocity =
    // drivebase.driveFieldOriented(driveRobotOriented);
    // Command driveSetpointGen =
    // drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    // Command driveFieldOrientedAnglularVelocityKeyboard =
    // drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    // Command driveSetpointGenKeyboard =
    // drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    }
    
    // -----------------------------------------------------------------------Default Commands-----------------------------------------------------------------------
    conveyor.setDefaultCommand(conveyor.clearCoral(coralPresent, elevator));
    lights.setDefaultCommand(lights.set(Lights.Special.OFF));
    elevator.setDefaultCommand(elevator.idle(this::isArmInsideElevator, sensation::clawCoralPresent).repeatedly());
    sensation.setDefaultCommand(Commands.idle(sensation));
    arm.setDefaultCommand(arm.idle(this::isSafeForArmToMoveUp, this::isSafeForArmToMoveDown, sensation::clawCoralPresent).repeatedly());

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      driveDirectAngleKeyboard.driveToPose(
          () -> target,
          new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
          new ProfiledPIDController(5, 0, 0,
              new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
          () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
    }

    if (DriverStation.isTest()) {
      // drivebase.setDefaultCommand(driveFieldOrienteAnglularVelocity); // Overrides
      // drive command above!
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.a().onTrue(new Clear(arm));
      driverXbox.x().onTrue(new L3(elevator, arm));
      driverXbox.b().onTrue(new Collecting(elevator, arm, sensation));
      driverXbox.y().onTrue(new L4(elevator, arm));
      driverXbox.leftBumper().whileTrue(new ScoreL4(arm, drivebase));
      driverXbox.rightBumper().whileTrue(new MoveToTarget(this));
      driverXbox.leftTrigger().whileTrue(new CenterAndScore(this, true));
      driverXbox.rightTrigger().whileTrue(new CenterAndScore(this, false));
      driverXbox.povUp().whileTrue(climber.ascend().repeatedly());
      driverXbox.povDown().whileTrue(climber.descend().repeatedly());
      driverXbox.povLeft().whileTrue(new SetArm(arm, ArmState.STOP).alongWith(new SetElevator(elevator, ElevatorState.STOP)));

      //copilot buttons

      copilotXbox.rightBumper().onTrue(new InstantCommand(()->{targetedSide = TargetSide.RIGHT;} ));
      copilotXbox.leftBumper().onTrue(new InstantCommand(()->{targetedSide = TargetSide.LEFT;}));

      // copilot poses blue
      copilotButtons.button(1).and(isRedAllianceTrigger.negate()).onTrue(new SetTargetPose(this, 18));
      copilotButtons.button(2).and(isRedAllianceTrigger.negate()).onTrue(new SetTargetPose(this, 17));
      copilotButtons.button(3).and(isRedAllianceTrigger.negate()).onTrue(new SetTargetPose(this, 22));
      copilotButtons.button(4).and(isRedAllianceTrigger.negate()).onTrue(new SetTargetPose(this, 21));
      copilotButtons.button(5).and(isRedAllianceTrigger.negate()).onTrue(new SetTargetPose(this, 20));
      copilotButtons.button(6).and(isRedAllianceTrigger.negate()).onTrue(new SetTargetPose(this, 19));

      // copilot poses red
      copilotButtons.button(1).and(isRedAllianceTrigger).onTrue(new SetTargetPose(this, 7));
      copilotButtons.button(2).and(isRedAllianceTrigger).onTrue(new SetTargetPose(this, 8));
      copilotButtons.button(3).and(isRedAllianceTrigger).onTrue(new SetTargetPose(this, 9));
      copilotButtons.button(4).and(isRedAllianceTrigger).onTrue(new SetTargetPose(this, 10));
      copilotButtons.button(5).and(isRedAllianceTrigger).onTrue(new SetTargetPose(this, 11));
      copilotButtons.button(6).and(isRedAllianceTrigger).onTrue(new SetTargetPose(this, 6));
      

      // TRIGGERS

      robotInPosition.whileTrue(lights.set(Lights.Colors.GREEN, Lights.Patterns.SOLID));
      coralPresent.and(coralExit.negate()).and(coralHopper.negate())
          .onTrue(lights.set(Lights.Colors.RED, Lights.Patterns.FAST_FLASH));
      coralHopper.and(coralExit.negate()).onTrue(lights.set(Lights.Colors.RED, Lights.Patterns.MARCH));
      coralExit.onFalse(lights.set(Lights.Colors.RED, Lights.Patterns.SOLID));
      isTeleopTrigger.and(coralExit).and(readyToCollectTrigger).onTrue(new CollectAndPost(elevator, arm, sensation));
    }
  }

  public static boolean isRed() {
    try {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    } catch (NoSuchElementException e) {
      return false;
    }
  }

  public void setTargetPose(int targetAprilTagID) {
    this.targetAprilTagID = targetAprilTagID;
  }

  public int getTargetAprilTagID() {
    return targetAprilTagID;
  }

  public boolean isSafeForArmToMoveUp() {
    double safeCarriagePosition = 60.0;
    double safeArmAngle = 150;
    //don't move up if just collected coral and the elevator has not moved up yet to get the coral free from cradle
    return elevator.getCarriagePosition() < safeCarriagePosition || arm.getPosition() > safeArmAngle;
  }

  public boolean isSafeForArmToMoveDown() {
    double maxUnsafeDistance = 60.0;
    return elevator.getCarriagePosition() > maxUnsafeDistance;
  }

  public boolean isArmInsideElevator() {
    return elevator.getCarriagePosition() > 200 && arm.getPosition() < 35;
  }

  public boolean readyToCollect() {
    return elevator.hasReachedPosition(ElevatorState.SAFE_CORAL) && arm.matchesState(ArmState.COLLECT);
  }

  public boolean isManualControlMode() {
    return isManualControlMode();
  }

  public void teleopInit() {
    drivebase.setMotorBrake(true);
    arm.setState(ArmState.START);
  }

  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }

  public Arm getArm() {
    return arm;
  }

  public Elevator getElevator() {
    return elevator;
  }

  public Conveyor getConveyor() {
    return conveyor;
  }

  public Lights getLights() {
    return lights;
  }

  public Sensation getSensation() {
    return sensation;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public Pose2d scorePose(int aprilTagID, TargetSide side) {
    Optional<Pose3d> aprilTagPose3d = Vision.fieldLayout.getTagPose(aprilTagID);
    if (!aprilTagPose3d.isPresent())
    {
      return new Pose2d();
    }

    Pose2d apPose = aprilTagPose3d.get().toPose2d();

    double xMeters = 0.7;
    double yMeters = (side == TargetSide.LEFT ? -0.1 : 0.1);

        Translation2d localOffset = new Translation2d(xMeters, yMeters); // right is negative Y

        // Rotate the local offset into the global coordinate frame
        Translation2d rotatedOffset = localOffset.rotateBy(apPose.getRotation());

    // Compute the new position
    Translation2d newTranslation = apPose.getTranslation().plus(rotatedOffset);

    return new Pose2d(newTranslation, apPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
  }

  public boolean inPosition() {
    return sensation.leftTOFisValid() &&
        sensation.rightTOFisValid() &&
        sensation.leftTOF() < 1000 &&
        sensation.rightTOF() < 1000;
    // robotController.visionSystem.getDoubleCameraReefApriltag() != -1;
  }

}

