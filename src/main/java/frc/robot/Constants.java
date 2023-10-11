// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SPI.Port;

public class Constants {
  public static SimMode simMode = SimMode.DESKTOP;

  public static class kSwerve {
    // Chassis dimensions from wheel center to center (meters)
    public static double width = 0.5;
    public static double length = 0.5;

    // Speed & accel limits (m/s, rad/s, m/s^2 & rad/s^2)
    public static double maxTransSpeed = 5;
    public static double maxAngSpeed = 2 * Math.PI;

    public static double maxTransAccel = 1.2;
    public static double maxAngAccel = 1.2;

    // Operator interface constants
    public static class OI {
      public static double translationGain = 0.5;
      public static double rotationGain = 0.5;

      public static boolean closedLoop = false;
    }

    // Swerve uses ccw+ angular quanities and a coordinate plane with 0,0 at the robot's center
    // , forward is +x, and a module order based on the quadrant system (front left is first)
    public static SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(length / 2, width / 2),
            new Translation2d(-length / 2, width / 2),
            new Translation2d(-length / 2, -width / 2),
            new Translation2d(length / 2, -width / 2));

    // Module angular offsets (rad)
    public static class Offsets {
      public static double frontLeft = 0;
      public static double backLeft = Math.PI / 2;
      public static double backRight = Math.PI;
      public static double frontRight = -Math.PI / 2;
    }

    public static boolean invertGyro = false;

    // Controller PID values for x/y translation, and z rotation
    public static class Auton {
      public static final double xP = 0;
      public static final double xI = 0;
      public static final double xD = 0;

      public static final double yP = 0;
      public static final double yI = 0;
      public static final double yD = 0;

      public static final double zP = 0;
      public static final double zI = 0;
      public static final double zD = 0;

      public static final double maxAccel = 2;
      public static final double maxVel = 2;
    }

    public static class kModule {
      // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
      public static final int drivingMotorPinionTeeth = 14;

      public static final boolean steeringEncoderInverted = true;

      // Physical dimensions/values
      public static final double kWheelDiameterMeters = 0.0762;
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
      public static final double drivingMotorReduction =
          (45.0 * 22) / (drivingMotorPinionTeeth * 15);
      public static final double steeringMotorReduction = 1;
      public static final double maxDriveSpeed = 4;

      // Encoders
      public static final double drivingEncoderPositionFactor =
          (kWheelDiameterMeters * Math.PI) / drivingMotorReduction; // meters
      public static final double drivingEncoderVelocityFactor =
          ((kWheelDiameterMeters * Math.PI) / drivingMotorReduction) / 60.0; // meters per second

      public static final double steeringEncoderPositionFactor = (2 * Math.PI); // radians
      public static final double steeringEncoderVelocityFactor =
          (2 * Math.PI) / 60.0; // radians per second

      public static final double steeringEncoderPositionPIDMinInput = 0; // radians
      public static final double steeringEncoderPositionPIDMaxInput =
          steeringEncoderPositionFactor; // radians

      // PID constants
      public static final double drivingP = 0.04;
      public static final double drivingI = 0;
      public static final double drivingD = 0;
      public static final double drivingS = 0;
      public static final double drivingV = 0;
      public static final double drivingA = 0;
      public static final double drivingMinOutput = -1;
      public static final double drivingMaxOutput = 1;

      public static final double steeringP = 1;
      public static final double steeringI = 0;
      public static final double steeringD = 0;
      public static final double steeringS = 0;
      public static final double steeringV = 0;
      public static final double steeringA = 0;
      public static final double steeringMinOutput = -1;
      public static final double steeringMaxOutput = 1;

      // Motor configs
      public static final IdleMode drivingMotorIdleMode = IdleMode.kBrake;
      public static final IdleMode steeringMotorIdleMode = IdleMode.kBrake;

      public static final int drivingMotorCurrentLimit = 50; // amps
      public static final int steeringMotorCurrentLimit = 20; // amps
    }

    // Motor CAN IDs
    public static class CANID {
      public static int frontLeftDrive = 2;
      public static int frontLeftSteer = 3;
      public static int backLeftDrive = 4;
      public static int backLeftSteer = 5;
      public static int backRightDrive = 6;
      public static int backRightSteer = 7;
      public static int frontRightDrive = 8;
      public static int frontRightSteer = 9;
    }

    public static Port navxPort = Port.kMXP;
  }

  public enum SimMode {
    HARDWARE,
    DESKTOP_VISION,
    DESKTOP
  }
}
