// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class ModuleConstants {
        //Physical
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.8);
        public static final double kDriveMotorGearRatio = 6.75;
        public static final double kTurningMotorGearRatio = 21.4286;

        //Conversions
        public static final double kFalconTicks = 2048;
        public static final double kRadiansToFalcon = kFalconTicks / (2 * Math.PI);
        public static final double kRadiansToTurning = kRadiansToFalcon * kTurningMotorGearRatio;
        public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
        public static final double kMetersToRotations = (1 / kWheelCircumference);
        public static final double kMetersToDrive = kMetersToRotations * kDriveMotorGearRatio * kFalconTicks;
        public static final double kMetersToDriveVelocity = kMetersToDrive / 10;
        public static final double kAbsToRadians = 2.0 * Math.PI;

        //Gains
            //Turn
            public static final double kPTurning = 0.21; //0.21 //works from 0.1-0.3 but 0.21 seems to offer low chattering and pretty quick alignment
            //Drive
            public static final double kAFFDrive = 0.015; //0.0151 //0.015
            public static final double kFDrive = .045012; //0.04390375 //0.03751 //.045012
            public static final double kPDrive = 0.02; //0.08 //0.02
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(22.5); //need to find
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22.5); //need to find


        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        );

        //TBD
        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kFrontRightDriveMotorPort = 2;
        public static final int kBackLeftDriveMotorPort = 3;
        public static final int kBackRightDriveMotorPort = 4;

        //TBD
        public static final int kFrontLeftTurningMotorPort = 5;
        public static final int kFrontRightTurningMotorPort = 6;
        public static final int kBackLeftTurningMotorPort = 7;
        public static final int kBackRightTurningMotorPort = 8;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        //TBD
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        //sort of calculated
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.613 * Math.PI * 2;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.536 * Math.PI * 2;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -0.605 * Math.PI * 2;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -0.719 * Math.PI * 2;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kRadius = Units.inchesToMeters(32/2);

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.8; //13.3 adjusted, 16.4 free //empirical 4.8 meters when not on ground
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / kRadius;

        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = Units.feetToMeters(100); //10 //100 for testing
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * 2 * Math.PI; //idk
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;

        public static final double kDegreesToRadians = (2*Math.PI) / 360;

        public static final double kPTurning = 0.0015; //0.0015 low-no oscillation
        public static final double kDTurning = 0.0; //0.0 unnecissary

        public static final double kPFudge = 0.02; //0.2 seems pretty close
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kDriverYAxis = XboxController.Axis.kLeftY.value;
        public static final int kDriverXAxis = XboxController.Axis.kLeftX.value;
        public static final int kDriverRotAxis = XboxController.Axis.kRightX.value;
        public static final int kDriverFieldOrientedButtonIdx = XboxController.Button.kRightBumper.value;

        public static final double kDeadband = 0.06; //0.0275-0.03 //0.06
    }

    public static final class PathingConstants{
        public static final double kRobotWidth = DriveConstants.kTrackWidth;
        public static final double kRobotLength = DriveConstants.kWheelBase;
        public static final double kRobotRadius = Math.sqrt(Math.pow(kRobotWidth/2,2)+Math.pow(kRobotLength/2,2));

    }

    public static final class VisionConstants {
        public static final double kMaxAmbiguity = 0.2;
    }
}
