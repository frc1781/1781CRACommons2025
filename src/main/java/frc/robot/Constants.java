// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{


  public static final Vision USING_VISION = Vision.PHOTON_VISION;  //set true if you have working vision
  public static final boolean UPDATE_HEADING_FROM_VISION = false;  //if false heading is only from gyro
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class TankDrivebaseConstants
  {
    // CAN IDs
    public static final int LEFT_1  = 1;
    public static final int LEFT_2  = 2;
    public static final int RIGHT_1 = 3;
    public static final int RIGHT_2 = 4;
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class SensationConstants
  {
    public static double ARM_TOF_DISTANCE = 700;
    public static double MAX_TIME_LOOKING_FOR_POLE = 0.5;

    public static int ARM_TOF_ID = 51;
    public static int RIGHT_FRONT_TOF_ID = 52;
    public static int LEFT_FRONT_TOF_ID = 53;

    public static double TARGET_TOF_PARALLEL_DISTANCE = 280;
    public static double TARGET_TOF_PARALLEL_DISTANCE_SHORT = 170;
    public static double TARGET_TOF_CENTERING_PARALLEL_DISTANCE = 250;

    public static final int enter = 3;
    public static final int hopperBack = 1;
    public static final int hopperFront = 0;
    public static final int exit = 2;
  }

  public static class Conveyor {
    public static final int MOTOR_CAN_ID = 15;
    public static final int CURRENT_LIMIT = 30;
  }

  public static class Climber {
        public static final int MOTOR = 14;

        public static final double RADIANS_PER_REVOLUTION = (Math.PI * 2) / 125;

        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;

        public static final double KS = 0;
        public static final double KG = 3.96;
        public static final double KV = 2.44;
        public static final double KA = 0.14;

        public static final ClosedLoopConfig CLOSED_LOOP_CONFIG = new ClosedLoopConfig()
                .p(P)
                .i(I)
                .d(D);
  }

  public static class Elevator {
        public static final int RIGHT_ELEVATOR_MOTOR = 12;
        public static final int LEFT_ELEVATOR_MOTOR = 11;
        public static final int FRAME_TOF = 58;
        public static final int CARRIAGE_TOF = 57;
        // https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A30%2C%22u%22%3A%22A%22%7D&efficiency=100&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A5.93175%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A9%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A35%2C%22u%22%3A%22in%22%7D
        public static final double MAX_ELEVATION_MPS = 0.87;
        public static final double ELEVATOR_KS = 0.02; // KS cannot be modeled and needs to be measured
        // public static final double ELEVATOR_KG = 0.13;
        // public static final double ELEVATOR_KV = 13.81;
        // Changing these back because may be causing the rope to break
        public static final double ELEVATOR_KG = 0.07;
        public static final double ELEVATOR_KV = 4.60;
        public static final double ELEVATOR_KA = 0.01;
  }

  public static class Arm {
        public static final int ARM_MOTOR_ID = 13;
        public static final int CLAW_CORAL_SENSOR_ID = 54;
  }
  
  public enum Vision {
    NO_VISION,
    PHOTON_VISION,
    LIMELIGHT_VISION
  }

}