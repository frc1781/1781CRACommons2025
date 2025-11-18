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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.lang.ModuleLayer.Controller;

import javax.sound.sampled.Port;

import swervelib.SwerveInputStream;

public class RobotContainer
{
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandPS4Controller driverPS = new CommandPS4Controller(0);
  //private final Sensation sensation = new Sensation();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/robot"));
  // private final TankDriveTrain tankDrive = new TankDriveTrain(driverXbox);
  // private final Conveyor conveyor = new Conveyor();
  // private final Lights lights = new Lights();
  // private final Climber climber = new Climber();
  private final SendableChooser<Command> autoChooser;
  private double wait_seconds = 5;

  //Trigger coralEnter = new Trigger(sensation::coralPresent);
 // Trigger coralHopper = new Trigger(sensation::coralInHopper);
 // Trigger coralExit = new Trigger(sensation::coralExitedHopper);

  //Driving the robot during teleOp
  SwerveInputStream driveAngularVelocityXBox = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
    () -> driverXbox.getLeftY() * -1,
    () -> driverXbox.getLeftX() * -1)
    .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)  
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)  //might be changed to 1
    .allianceRelativeControl(true)
    .cubeRotationControllerAxis(true);

    SwerveInputStream driveAngularVelocityPS = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> driverPS.getLeftY() * -1,
      () -> driverPS.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverPS.getRightX() * -1)  
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)  //might be changed to 1
      .allianceRelativeControl(true)
      .cubeRotationControllerAxis(true);

  //Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
  SwerveInputStream driveDirectAngleXBox = driveAngularVelocityXBox.copy()
    .withControllerHeadingAxis(() -> driverXbox.getRightX() * -1, () -> driverXbox.getRightY() * -1)
    .headingWhile(true);

    SwerveInputStream driveDirectAnglePS = driveAngularVelocityPS.copy()
    .withControllerHeadingAxis(() -> driverPS.getRightX() * -1, () -> driverPS.getRightY() * -1)
    .headingWhile(true);

   // Clone's the angular velocity input stream and converts it to a robotRelative input stream.
  SwerveInputStream driveRobotOrientedXBox = driveAngularVelocityXBox.copy()
    .robotRelative(true)
    .allianceRelativeControl(false);

    SwerveInputStream driveRobotOrientedPS = driveAngularVelocityPS.copy()
    .robotRelative(true)
    .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboardXBox = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
    () -> -driverXbox.getLeftY(),
    () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis( 2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocityKeyboardPS = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
    () -> -driverPS.getLeftY(),
    () -> -driverPS.getLeftX())
      .withControllerRotationAxis(() -> driverPS.getRawAxis( 2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleKeyboardXBox = driveAngularVelocityKeyboardXBox.copy()
    .withControllerHeadingAxis(
      () -> Math.sin(driverPS.getRawAxis(2) * Math.PI) *(Math.PI *2),
      () -> Math.cos(driverPS.getRawAxis(2) *Math.PI) *(Math.PI *2))
        .headingWhile(true)
        .translationHeadingOffset(true)
        .translationHeadingOffset(Rotation2d.fromDegrees( 0));

  SwerveInputStream driveDirectAngleKeyboardPS = driveAngularVelocityKeyboardPS.copy()
    .withControllerHeadingAxis(
      () -> Math.sin(driverPS.getRawAxis(2) * Math.PI) *(Math.PI *2),
      () -> Math.cos(driverPS.getRawAxis(2) *Math.PI) *(Math.PI *2))
        .headingWhile(true)
        .translationHeadingOffset(true)
        .translationHeadingOffset(Rotation2d.fromDegrees( 0));

  public RobotContainer()
  {
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("CustomWaitCommand", new WaitCommand(SmartDashboard.getNumber("Wait Time", wait_seconds)));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putNumber("Wait Time", wait_seconds);
  }

  private void configureBindings()
  {
    if(driverXbox.isConnected()){
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngleXBox);
  }else{Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAnglePS);}
    //Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    //Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    //Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    if(driverXbox.isConnected()){
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboardXBox);
    }else{Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboardPS);}
    //Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    //Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      if (driverXbox.isConnected()) { 
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboardXBox);
      } else{ drivebase.setDefaultCommand(driveAngularDirectAngleKeyboardPS);}
    } 
    else
    {

      if (driverXbox.isConnected()) { 
        drivebase.setDefaultCommand(driveFieldOrientedDirectAngleXBox);
        } else{ 
      drivebase.setDefaultCommand(driveFieldOrientedDirectAnglePS);
    }
  }

    //conveyor.setDefaultCommand(conveyor.clearCoral(coralHopper));
   // lights.setDefaultCommand(lights.set(Lights.Special.OFF));
   // climber.setDefaultCommand(climber.idle());

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      if(driverXbox.isConnected()){
      driveDirectAngleKeyboardXBox.driveToPose(
        () -> target, 
        new ProfiledPIDController(5, 0,0, new Constraints(5, 2)),
        new ProfiledPIDController(5,0,0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboardXBox.driveToPoseEnabled(true),
                                                      () -> driveDirectAngleKeyboardXBox.driveToPoseEnabled(false)));
      } else{
        driveDirectAngleKeyboardPS.driveToPose(
          () -> target, 
          new ProfiledPIDController(5, 0,0, new Constraints(5, 2)),
          new ProfiledPIDController(5,0,0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
        driverPS.options().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
        driverPS.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
        driverPS.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboardPS.driveToPoseEnabled(true),
                                                        () -> driveDirectAngleKeyboardPS.driveToPoseEnabled(false)));
      }
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

      driverPS.square().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverPS.triangle().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverPS.options().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverPS.share().whileTrue(drivebase.centerModulesCommand());
      driverPS.L1().onTrue(Commands.none());
      driverPS.R1().onTrue(Commands.none());
    } 
    else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
     // driverXbox.povUp().whileTrue(climber.ascend());
     // driverXbox.povDown().whileTrue(climber.descend());
     // driverXbox.y().onTrue(lights.set(Lights.Special.RAINBOW));
     // driverXbox.b().onTrue(lights.set(Lights.Colors.WHITE, Lights.Patterns.MARCH));

      driverPS.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverPS.square().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverPS.options().whileTrue(Commands.none());
      driverPS.share().whileTrue(Commands.none());
      driverPS.L1().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverPS.R1().onTrue(Commands.none());
      // driverPS.povUp().whileTrue(climber.ascend());
      // driverPS.povDown().whileTrue(climber.descend());
      // driverPS.triangle().onTrue(lights.set(Lights.Special.RAINBOW));
      // driverPS.circle().onTrue(lights.set(Lights.Colors.WHITE, Lights.Patterns.MARCH));

      //coralEnter.and(coralExit.negate()).and(coralHopper.negate()).onTrue(lights.set(Lights.Colors.RED, Lights.Patterns.FAST_FLASH));
     // coralHopper.and(coralExit.negate()).onTrue(lights.set(Lights.Colors.RED, Lights.Patterns.MARCH));
     // coralExit.onFalse(lights.set(Lights.Colors.RED, Lights.Patterns.SOLID));
    }
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
}