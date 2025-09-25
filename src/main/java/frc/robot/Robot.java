// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.PowerDistribution;

import java.sql.Driver;

import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command exampleAuto;
  private RobotContainer theRobotContainer;
  private Timer disabledTimer;

  public RobotContainer robotContainer() {
    return theRobotContainer;
  }
  
  public Robot getInstance() {
    return this;
  }

  public void robotInit() {
    theRobotContainer = new RobotContainer();
    disabledTimer = new Timer(); //for turning off breaking when disabled

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();

    if (isSimulation())  {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    theRobotContainer.periodic();
  }

  @Override
  public void disabledInit() {
    theRobotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      theRobotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  @Override
  public void autonomousInit() {
    theRobotContainer.setMotorBrake(true);
    exampleAuto = theRobotContainer.getAutonomousCommand();

    if(exampleAuto.getName().equals("StandardLeft")) {
      if (RobotContainer.isRed()) {
        theRobotContainer.getDrivebase().resetOdometry(Constants.Positions.getPositionForRobot(101));
      } 
      else {
        theRobotContainer.getDrivebase().resetOdometry(Constants.Positions.getPositionForRobot(201));
      }
    }

    if(exampleAuto.getName().equals("StandardRight")) {
      if (RobotContainer.isRed()) {
        theRobotContainer.getDrivebase().resetOdometry(Constants.Positions.getPositionForRobot(102));
      } 
      else {
      }
    }
    

    if (exampleAuto != null) {
      exampleAuto.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    theRobotContainer.teleopInit();
    if (exampleAuto != null) {
      exampleAuto.cancel();
    } 
    else {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit()  {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
