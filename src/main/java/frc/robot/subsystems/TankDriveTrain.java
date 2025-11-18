// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

public class TankDriveTrain extends SubsystemBase {
  TalonFX left1;
  TalonFX left2;

  TalonFX right1;
  TalonFX right2;

  DifferentialDrive drive;
  CommandPS4Controller driverPS;
  CommandXboxController driverXbox;

  /** Creates a new TankDriveTrain. */
  public TankDriveTrain() {
    left1 = new TalonFX(Constants.TankDrivebaseConstants.LEFT_1);
    left2 = new TalonFX(Constants.TankDrivebaseConstants.LEFT_2);
    right1 = new TalonFX(Constants.TankDrivebaseConstants.RIGHT_1);
    right2 = new TalonFX(Constants.TankDrivebaseConstants.RIGHT_2);
    configureMotors();

    drive = new DifferentialDrive(left1::set, right1::set);
  }

  public TankDriveTrain(CommandXboxController driverXbox) {
    this();
    this.driverXbox = driverXbox;
}

public TankDriveTrain(CommandPS4Controller driverPS) {
    this();
    this.driverPS = driverPS;
}
  
  public void configureMotors() {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    
    // Not inverted and brake
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Inverted and brake
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    left1.getConfigurator().apply(leftConfig);
    left2.getConfigurator().apply(leftConfig);
    right1.getConfigurator().apply(rightConfig);
    right2.getConfigurator().apply(rightConfig);

    left2.setControl(new Follower(left1.getDeviceID(), false));
    right2.setControl(new Follower(right1.getDeviceID(), false));
  }

  @Override
  public void periodic() {
      double forward = 0;
      double turn = 0;
  
      // Pick whichever controller was supplied in the constructor
      if (driverXbox != null) {
          forward = -driverXbox.getLeftY();
          turn = driverXbox.getRightX();
      } 
      else {
          forward = -driverPS.getLeftY();
          turn = driverPS.getRightX();
      }
  
      // Deadband (optional)
      if (Math.abs(forward) < 0.1) forward = 0;
      if (Math.abs(turn) < 0.1)    turn = 0;
  
      drive.arcadeDrive(forward, turn);
  }
  
}
