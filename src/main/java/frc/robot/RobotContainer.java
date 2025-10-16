// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TargetSide;
import frc.robot.commands.CenterAndScoreL3;
import frc.robot.commands.CenterAndScoreL4;
import frc.robot.commands.Clear;
import frc.robot.commands.Collect;
import frc.robot.commands.CollectAndClear;
import frc.robot.commands.L2;
import frc.robot.commands.L2hold;
import frc.robot.commands.L3;
import frc.robot.commands.L4;
import frc.robot.commands.L4hold;
import frc.robot.commands.MoveBack;
import frc.robot.commands.MoveToTarget;
import frc.robot.commands.PostCollect;
import frc.robot.commands.PreCollect;
import frc.robot.commands.PreCollectAuto;
import frc.robot.commands.SafeConfig;
import frc.robot.commands.ScoreL4;
import frc.robot.commands.ScoreLow;
import frc.robot.commands.SetArm;
import frc.robot.commands.SetElevator;
import frc.robot.commands.SetTargetPose;
//import frc.robot.commands.StopMovingToTarget;
import frc.robot.commands.StrafeCommand;
import frc.robot.commands.WaitForCoral;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Sensation;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private String robotPoseHasBeenSetFor = "nothing"; 
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController copilotXbox = new CommandXboxController(1);
  final CommandJoystick copilotButtons = new CommandJoystick(2);
  //private Sensation sensation = new Sensation();
  private SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/ralph"));
  //private final Conveyor conveyor = new Conveyor();
  private final Lights lights = new Lights();
  //private final Elevator elevator = new Elevator(this);
  //private final Arm arm = new Arm(this);
  //private final Climber climber = new Climber();
  //private final SendableChooser<Command> autoChooser;
  private double wait_seconds = 5;
  public int targetAprilTagID = -1;
  public TargetSide targetedSide = TargetSide.LEFT;
  private boolean sendyPressed = false;

  // Trigger coralPresent = new Trigger(sensation::coralPresent);
  // Trigger coralHopper = new Trigger(sensation::coralInHopper);
  // Trigger coralExit = new Trigger(sensation::coralExitedHopper);
  // Trigger armHasCoral = new Trigger(sensation::clawCoralPresent);
  // Trigger robotInPosition = new Trigger(this::inPosition);
  // Trigger readyToCollectTrigger = new Trigger(this::readyToCollect);
  // Trigger isTeleopTrigger = new Trigger(DriverStation::isTeleop);
  // Trigger isRedAllianceTrigger = new Trigger(RobotContainer::isRed);
  // Trigger coralInClaw = new Trigger(sensation::clawCoralPresent);
  // Trigger targetAquired = new Trigger(()->{return targetAprilTagID != -1;});
  // Trigger seeingReefPole = new Trigger(sensation::armTOFisValid);
 

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
       driverJoystickX(),
       driverJoystickY())
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
    // NamedCommands.registerCommand("Score", new ScoreL4(arm, drivebase));
    // NamedCommands.registerCommand("Collect", new CollectAndClear(elevator, arm, sensation));
    // NamedCommands.registerCommand("MoveToPositionToScore", drivebase.new MoveToPositionToScore(sensation));
    // NamedCommands.registerCommand("Clear", new Clear(arm));
    // NamedCommands.registerCommand("StrafeCommand", new StrafeCommand(drivebase, elevator, arm, sensation, () -> true));
    // NamedCommands.registerCommand("L4", new L4(elevator, arm));
    // NamedCommands.registerCommand("PreCollect", new PreCollectAuto(elevator, arm, sensation));
    // NamedCommands.registerCommand("PostCollect", new PostCollect(elevator, arm));
    // NamedCommands.registerCommand("SetElevator", new SetElevator(elevator, ElevatorState.SAFE));
    // NamedCommands.registerCommand("SetArm", new SetArm(arm, ArmState.START));
    // NamedCommands.registerCommand("SafeConfig", new SafeConfig(elevator, arm));
    // NamedCommands.registerCommand("MoveBack", new MoveBack(drivebase));
    // NamedCommands.registerCommand("WaitForCoral", new WaitForCoral(sensation, drivebase));

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    // NamedCommands.registerCommand("CustomWaitCommand",
    //     new WaitCommand(SmartDashboard.getNumber("Wait Time", wait_seconds)));
    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", autoChooser);

    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      Logger.recordOutput("Drive/currentPose", pose);
    });

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      Logger.recordOutput("Drive/targetPose", pose);
    });

    // PathPlannerLogging.setLogActivePathCallback((path) -> {
    //   if (path != null) {
    //     Logger.recordOutput("Drive/CurrentCommand", "RunningPath");
    //   } 
    // });
  }

  public void periodic() {
    SmartDashboard.getNumber("Target Apriltag", targetAprilTagID);
    // Logger.recordOutput("RobotContainer/isSafeForArmToMoveUp", isSafeForArmToMoveUp());
    // Logger.recordOutput("RobotContainer/isSafeForArmToMoveDown", isSafeForArmToMoveDown());
    // Logger.recordOutput("RobotContainer/isArmInsideElevator", isArmInsideElevator());
    // Logger.recordOutput("RobotContainer/readyToCollect", readyToCollect());

    //APRILTAGS
    aquireTargetAprilTag();
    Logger.recordOutput("RobotContainer/targetAprilTagID", targetAprilTagID);
    Logger.recordOutput("RobotContainer/targetPose", scorePose(targetAprilTagID, targetedSide));
    Logger.recordOutput("targetedSide", targetedSide.toString());

  }

  private void configureBindings() {
    //Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
     Command driveFieldOrientedAngularVelocity =
    drivebase.driveFieldOriented(driveAngularVelocity);
    // Command driveRobotOrientedAngularVelocity =
    // drivebase.driveFieldOriented(driveRobotOriented);
    // Command driveSetpointGen =
    // drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    // Command driveFieldOrientedAnglularVelocityKeyboard =
    // drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    // Command driveSetpointGenKeyboard =
    // drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    // -----------------------------------------------------------------------Default Commands-----------------------------------------------------------------------
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    //conveyor.setDefaultCommand(conveyor.clearCoral(coralPresent, elevator));
    lights.setDefaultCommand(lights.set(Lights.Special.OFF));
    // elevator.setDefaultCommand(elevator.idle(this::isArmInsideElevator, sensation::clawCoralPresent).repeatedly());
    // sensation.setDefaultCommand(Commands.idle(sensation));
    // arm.setDefaultCommand(
    //     arm.idle(this::isSafeForArmToMoveUp, this::isSafeForArmToMoveDown, sensation::clawCoralPresent).repeatedly());

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
      // driverXbox.a().onTrue(new Collect(elevator, arm, sensation));
      // driverXbox.x().onTrue(new L3(elevator, arm));
      // driverXbox.b().onTrue(new PreCollect(elevator, arm, sensation));
      //driverXbox.b().onTrue(new Collecting(elevator, arm, sensation));
      //driverXbox.y().onTrue(new L4(elevator, arm));
      //driverXbox.leftBumper().whileTrue(new ScoreL4(arm, drivebase));
      
     
      // driverXbox.povUp().whileTrue(climber.ascend().repeatedly());
      // driverXbox.povDown().whileTrue(climber.descend().repeatedly());
      // driverXbox.povLeft()
      //     .whileTrue(new SetArm(arm, ArmState.STOP).alongWith(new SetElevator(elevator, ElevatorState.STOP)));

      // copilot buttons
      // copilotXbox.leftBumper().whileTrue(new MoveToTarget(this, TargetSide.LEFT));
      // copilotXbox.rightBumper().whileTrue(new MoveToTarget(this, TargetSide.RIGHT));
      // copilotXbox.leftTrigger().whileTrue(new CenterAndScoreL4(this, () -> true));
      // copilotXbox.rightTrigger().whileTrue(new CenterAndScoreL4(this, () -> false));
      // copilotXbox.povLeft().whileTrue(new CenterAndScoreL3(this, () -> true));
      // copilotXbox.povRight().whileTrue(new CenterAndScoreL3(this, () -> false));
      // copilotXbox.y().whileTrue(new L4hold(elevator, arm));
      // copilotXbox.b().whileTrue(new L3(elevator, arm));
      // copilotXbox.a().whileTrue(new L2hold(elevator, arm));
      // copilotXbox.x().whileTrue(new ScoreLow(arm, drivebase));

      // copilot poses blue
      // copilotButtons.button(1).and(isRedAllianceTrigger.negate()).onTrue(new SetTargetPose(this, 18));
      // copilotButtons.button(2).and(isRedAllianceTrigger.negate()).onTrue(new SetTargetPose(this, 17));
      // copilotButtons.button(3).and(isRedAllianceTrigger.negate()).onTrue(new SetTargetPose(this, 22));
      // copilotButtons.button(4).and(isRedAllianceTrigger.negate()).onTrue(new SetTargetPose(this, 21));
      // copilotButtons.button(5).and(isRedAllianceTrigger.negate()).onTrue(new SetTargetPose(this, 20));
      // copilotButtons.button(6).and(isRedAllianceTrigger.negate()).onTrue(new SetTargetPose(this, 19));

      // copilot poses red
      // copilotButtons.button(1).and(isRedAllianceTrigger).onTrue(new SetTargetPose(this, 7));
      // copilotButtons.button(2).and(isRedAllianceTrigger).onTrue(new SetTargetPose(this, 8));
      // copilotButtons.button(3).and(isRedAllianceTrigger).onTrue(new SetTargetPose(this, 9));
      // copilotButtons.button(4).and(isRedAllianceTrigger).onTrue(new SetTargetPose(this, 10));
      // copilotButtons.button(5).and(isRedAllianceTrigger).onTrue(new SetTargetPose(this, 11));
      // copilotButtons.button(6).and(isRedAllianceTrigger).onTrue(new SetTargetPose(this, 6));

      // TRIGGERS

      // robotInPosition.whileTrue(lights.set(Lights.Colors.GREEN, Lights.Patterns.SOLID));
      
      // coralHopper.and(coralExit.negate()).onTrue(lights.set(Lights.Colors.RED, Lights.Patterns.MARCH));
      // coralExit.onFalse(lights.set(Lights.Colors.RED, Lights.Patterns.SOLID));
      // isTeleopTrigger.and(coralExit).and(readyToCollectTrigger).onTrue(new CollectAndPost(elevator, arm, sensation));
      // targetAquired.and(seeingReefPole.negate()).whileTrue(lights.set(Lights.Colors.RED, Lights.Patterns.SOLID));

      // seeingReefPole.and(armHasCoral).whileTrue(lights.set(Lights.Colors.RED, Lights.Patterns.FAST_FLASH));

      // armHasCoral.and(targetAquired.negate()).and(seeingReefPole.negate())
      //      .whileTrue(lights.set(Lights.Colors.GREEN, Lights.Patterns.SOLID));

      // coralPresent.and(armHasCoral.negate()).and(seeingReefPole.negate()).and(targetAquired.negate())
      //      .whileTrue(lights.set(Lights.Colors.GREEN, Lights.Patterns.FAST_FLASH));
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

  

  private void aquireTargetAprilTag() {
    List<Integer> aprilTagIDs = Vision.seenAprilTagIDs;
    double minimumDistance = Double.MAX_VALUE;
    int aprilTagID = -1;

    for (Integer i : aprilTagIDs) {
      if (drivebase.vision.getDistanceFromAprilTag(i) == -1 || i == -1) {
        continue;
      }
      Pose2d atPose = Vision.getAprilTagPose(i, Transform2d.kZero);
      if (atPose == null) {
        continue;
      }
      //filter out tags that are not on our side of the field
      if (isRed()) {
        if (i < 6 || i > 11) { //red reef tags are 6-11
          continue;
        }
      } else {
        if (i < 17 || i > 22) { //blue reef tags are 17-22
          continue;
        }
      }
      
      if (drivebase.vision.getDistanceFromAprilTag(i) < minimumDistance) // && 
      {
        minimumDistance = drivebase.vision.getDistanceFromAprilTag(i);
        aprilTagID = i;
      }

      Rotation2d aprilTagAngle = Vision.getAprilTagPose(aprilTagID, Transform2d.kZero).getRotation().rotateBy(Rotation2d.fromDegrees(180));
      if (!isRobotInSegment(aprilTagAngle, drivebase.getPose())) {  //disregard too oblique angles
        continue;
      }
    }
    targetAprilTagID = aprilTagID;  // -1 if none appropriate seen, still need to filter by only red or blue reef tags
  }

  //ONLY WORK FOR BLUE NEED TO FIX FOR RED
  private boolean isRobotInSegment(Rotation2d aprilTagFacing, Pose2d robotPose) {
  //find rotation from center of robot to, see if it is within 30 degrees from rotation of apriltag
    
    Translation2d centerOfReef = (isRed() ? (new Translation2d(13.04, 4.02)) : (new Translation2d(4.48, 4.02)));
    Translation2d robotTranslation = robotPose.getTranslation();
    Translation2d vectorToRobot = robotTranslation.minus(centerOfReef);
    Rotation2d angleToRobot = vectorToRobot.getAngle(); 
    Rotation2d angleDiff = angleToRobot.minus(aprilTagFacing.rotateBy(Rotation2d.fromDegrees(180)));
    if (Math.abs(angleDiff.getDegrees()) < 30) {
      return true;
    }
    return false;
  }


  public int getTargetAprilTagID() {
    return targetAprilTagID;
  }

  public boolean isSafeForArmToMoveUp() {
    // don't move up if just collected coral and the elevator has not moved up yet
    // to get the coral free from cradle
    return false;
  }

  public boolean isSafeForArmToMoveDown() {
    
    return false;
  }

  public boolean isArmInsideElevator() {
    return false;
  }

  public boolean readyToCollect() {
    return false;
  }

  public boolean isManualControlMode() {
    return isManualControlMode();
  }

  public boolean isElevatorUp () {
    return false;
  }

  public void teleopInit() {
    drivebase.setMotorBrake(true);
    //arm.setState(ArmState.START);
  }

  public DoubleSupplier driverJoystickX() { 
    if(copilotXbox.getHID().getLeftBumperButton() || 
    copilotXbox.getHID().getRightBumperButton() || 
    copilotXbox.getHID().getLeftTriggerAxis() > 0.1 || 
    copilotXbox.getHID().getRightTriggerAxis() > 0.1){
      return () -> 0;
    }
    
    // if(isElevatorUp()){
    //   return () -> driverXbox.getLeftX() * -0.1;
    // }
    return () -> driverXbox.getLeftX() * -1;
  }


  public DoubleSupplier driverJoystickY() {
    if(copilotXbox.getHID().getLeftBumperButton() || 
    copilotXbox.getHID().getRightBumperButton() || 
    copilotXbox.getHID().getLeftTriggerAxis() > 0.1 || 
    copilotXbox.getHID().getRightTriggerAxis() > 0.1){
      return () -> 0;
    }
    // if(isElevatorUp()){
    //   return () -> driverXbox.getLeftY() * -0.1;
    // }
    return () -> driverXbox.getLeftY() * -1;
  }

  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }

  public Arm getArm() {
    return null;
  }

  public Elevator getElevator() {
    return null;
  }

  public Conveyor getConveyor() {
    return null;
  }

  public Lights getLights() {
    return lights;
  }

  public Sensation getSensation() {
    return null;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   return autoChooser.getSelected();
  // }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public Pose2d scorePose(int aprilTagID, TargetSide side) {
    Optional<Pose3d> aprilTagPose3d = Vision.fieldLayout.getTagPose(aprilTagID);
    if (!aprilTagPose3d.isPresent()) {
      return new Pose2d();
    }

    Pose2d apPose = aprilTagPose3d.get().toPose2d();

    double xMeters = 0.8;
    double yMeters = (side == TargetSide.LEFT ? -0.1 : 0.1);

    Translation2d localOffset = new Translation2d(xMeters, yMeters); // right is negative Y

    // Rotate the local offset into the global coordinate frame
    Translation2d rotatedOffset = localOffset.rotateBy(apPose.getRotation());

    // Compute the new position
    Translation2d newTranslation = apPose.getTranslation().plus(rotatedOffset);

    return new Pose2d(newTranslation, apPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
  }

  // public boolean inPosition() {
  //   return sensation.leftTOFisValid() &&
  //       sensation.rightTOFisValid() &&
  //       sensation.leftTOF() < 1000 &&
  //       sensation.rightTOF() < 1000;
  //   // robotController.visionSystem.getDoubleCameraReefApriltag() != -1;
  // }

  // public void initializeRobotPositionBasedOnAutoRoutine(){
  //   Command autoroutine = getAutonomousCommand();
  //   String routineName = autoroutine.getName();

  //   if(robotPoseHasBeenSetFor.equals(routineName)) {
  //     return; //already set for this routine
  //   }

  //   getDrivebase().resetOdometry(Constants.Positions.getPositionForRobot(routineName));
  //   robotPoseHasBeenSetFor = routineName;
  // }
}
