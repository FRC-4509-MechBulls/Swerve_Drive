// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.FileReader;
import java.lang.reflect.Field;
import java.sql.Connection;
import java.sql.Time;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.lib.FieldTag;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort, 
          DriveConstants.kFrontLeftTurningMotorPort, 
          DriveConstants.kFrontLeftDriveEncoderReversed, 
          DriveConstants.kFrontLeftTurningEncoderReversed, 
          DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, 
          DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, 
          DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
  
  private final SwerveModule frontRight = new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort, 
          DriveConstants.kFrontRightTurningMotorPort, 
          DriveConstants.kFrontRightDriveEncoderReversed, 
          DriveConstants.kFrontRightTurningEncoderReversed, 
          DriveConstants.kFrontRightDriveAbsoluteEncoderPort, 
          DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, 
          DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

  private final SwerveModule backLeft = new SwerveModule(
          DriveConstants.kBackLeftDriveMotorPort, 
          DriveConstants.kBackLeftTurningMotorPort, 
          DriveConstants.kBackLeftDriveEncoderReversed, 
          DriveConstants.kBackLeftTurningEncoderReversed, 
          DriveConstants.kBackLeftDriveAbsoluteEncoderPort, 
          DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, 
          DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);


  private final SwerveModule backRight = new SwerveModule(
          DriveConstants.kBackRightDriveMotorPort, 
          DriveConstants.kBackRightTurningMotorPort, 
          DriveConstants.kBackRightDriveEncoderReversed, 
          DriveConstants.kBackRightTurningEncoderReversed, 
          DriveConstants.kBackRightDriveAbsoluteEncoderPort, 
          DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, 
          DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

  private Pigeon2 gyro = new Pigeon2(0);

  /*
  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
          new Rotation2d(gyro.getYaw(), gyro.getPitch()),gyro.getRoll(),
          new Pose2d(0,0,0),
          DriveConstants.kDriveKinematics,

  );
   */
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
          new Rotation2d(Math.toRadians(getHeading())), new Pose2d(0, 0, new Rotation2d()));
  Pose2d odometryPose = new Pose2d();


  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
        //put in thread so it doesn't stop the rest of our code from running
        new Thread(() -> {
                try {
                        Thread.sleep(1000);
                        gyro.configFactoryDefault();
                        gyro.configMountPose(AxisDirection.NegativeY, AxisDirection.PositiveZ);
                        zeroHeading();
                } catch (Exception e) {
                }
        
        }).start();
        //allows gyro to calibrate for 1 sec before requesting to reset^^

  }
  

  //reset gyroscope to have it set the current direction as the forward direction of field when robot boots up
  public void zeroHeading() {
      gyro.zeroGyroBiasNow();
      gyro.setYaw(0);
  }
    public void zeroHeading(double yaw) {
        gyro.zeroGyroBiasNow();
        gyro.setYaw(yaw);
    }

  //A number equal to x - (y Q), where Q is the quotient of x / y rounded to the nearest integer
  //(if x / y falls halfway between two integers, the even integer is returned)
  public double getHeading() {
        return Math.IEEEremainder(gyro.getYaw(), 360); //clamps value between -/+ 180 deg where zero is forward
  }

  //since wpilib often wants heading in format of Rotation2d
  public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
  }

  double lastDriveCall = 0;
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putBoolean("Drive method called", Timer.getFPGATimestamp()-lastDriveCall<0.1);

      m_odometry.update(new Rotation2d(Math.toRadians(getHeading())), frontLeft.getState(), frontRight.getState(),
              backLeft.getState(), backRight.getState());
      odometryPose = new Pose2d(m_odometry.getPoseMeters().getX()*6.75/0.66, m_odometry.getPoseMeters().getY()*6.75/0.66, m_odometry.getPoseMeters().getRotation());

      //SmartDashboard.putNumber("FL",frontLeft.getState().angle.getRadians());

      SmartDashboard.putNumber("oX",odometryPose.getX());
      SmartDashboard.putNumber("oY",odometryPose.getY());
  }

  public double[] getDesiredSpeeds(Pose2d pose){
      double[] out = new double[3];
      double rotationDiff = (pose.getRotation().getRadians() - odometryPose.getRotation().getRadians());
      double xDiff = (pose.getX() - odometryPose.getX());
      double yDiff = (pose.getY() - odometryPose.getY());
      double dist = Math.sqrt(Math.pow(xDiff,2) + Math.pow(yDiff,2));

      double dirToPose = Math.atan2(yDiff,xDiff);

      out[0] = dist * Math.cos(dirToPose +rotationDiff) *0.5;  // you might need to reverse sin and cos
      out[1] = dist * Math.sin(dirToPose+rotationDiff) * 0.5;
      out[2] = rotationDiff * 0.5;
      return out;
  }
  public void fieldTagSpotted(FieldTag fieldTag, Transform3d transform){
      double newX = -transform.getX() - fieldTag.getPose().getX();
      double newY = -transform.getY() - fieldTag.getPose().getY();


      Rotation2d newRotation = new Rotation2d((-transform.getRotation().getZ() - fieldTag.getPose().getRotation().getRadians())+2*Math.PI);


      SmartDashboard.putNumber("camHeading", newRotation.getRadians());
      SmartDashboard.putNumber("gyroHeading", (Math.toRadians(getHeading())+2*Math.PI)%(2*Math.PI));

      Pose2d newPose = new Pose2d(newX/6.75*0.66,newY/6.75*0.66, newRotation); //kil
      m_odometry.resetPosition(newPose, newRotation);
      zeroHeading(newRotation.getDegrees());

  }

  public void driveToPose(Pose2d pose){
      double[] speeds = getDesiredSpeeds(pose);

      speeds[0] = Constants.absMax(speeds[0],0.3);
      speeds[1] = Constants.absMax(speeds[1],0.3);
      speeds[2] = Constants.absMax(speeds[2],0.1);

      drive(speeds[0],speeds[1],speeds[2]);
  }

  public void resetPose(){
      m_odometry.resetPosition(new Pose2d(), new Rotation2d(Math.toRadians(getHeading())));
  }

  public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        //normalizes wheel speeds in case max speed reached^^
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
  }


  SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  SlewRateLimiter turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
      

boolean fieldOriented = false;
public boolean getFieldOriented(){return fieldOriented;}
public void setFieldOriented(boolean set){fieldOriented = set;}
public void toggleFieldOriented(){fieldOriented = !fieldOriented;}


  public void drive(double xSpeed, double ySpeed, double turningSpeed){
     //  Make the driving smoother, no sudden acceleration from sudden inputs
     xSpeed = xLimiter.calculate(xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
     ySpeed = yLimiter.calculate(ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
     turningSpeed = turningLimiter.calculate(turningSpeed * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
     //SmartDashboard.putNumber("xspeed", xSpeed);
     //SmartDashboard.putNumber("turningspeed", turningSpeed);
 
     // Construct desired chassis speeds (convert to appropriate reference frames)
     ChassisSpeeds chassisSpeeds;

      SmartDashboard.putBoolean("fieldOriented", fieldOriented);

      if(fieldOriented)
       chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, getRotation2d());
    else
       chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
     
 
     // Convert chassis speeds to individual module states
     SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
 
     // Output each module states to wheels
     setModuleStates(moduleStates);
  }

  public void joystickDrive(double xSpd, double ySpd, double turningSpd){
        
    // 1. Convert joystick inputs
    double xSpeed = xSpd;
    double ySpeed = ySpd;
    double turningSpeed = turningSpd;

    // 2. Apply deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
      drive(xSpeed, ySpeed, turningSpeed);
  }
       
}