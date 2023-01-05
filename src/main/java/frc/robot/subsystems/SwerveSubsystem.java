// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.lib.FieldTag;
import frc.robot.lib.MathThings;

public class SwerveSubsystem extends SubsystemBase {
  //Modules
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
      //Gyro
    private WPI_Pigeon2 gyro;

  //Odometry
    private SwerveDrivePoseEstimator odometry;
    private Pose2d initialPose;
    private ChassisSpeeds chassisSpeeds;

  //Values
    static boolean fieldOriented;
    
  //Dashboard
    //Tabs
    private ShuffleboardTab tabSwerveSubsystem;
    //Entries
    private NetworkTableEntry dashboardOdometryHeading;
    private NetworkTableEntry dashboardOdometryY;
    private NetworkTableEntry dashboardOdometryX;

  private double translationMagnitude;

  private double translationMagnitudeScaled;
  private double rotationMagnitude = 0;
  private double scaledMagnitudeRotation = 0;
  Rotation2d translationDirection;
  Rotation2d rotationDirection;


  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    //Odometry
      initialPose = new Pose2d();
    //modules
     frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort, 
         DriveConstants.kFrontLeftTurningMotorPort, 
         DriveConstants.kFrontLeftDriveEncoderReversed, 
         DriveConstants.kFrontLeftTurningEncoderReversed, 
         DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, 
         DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, 
         DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
     frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort, 
         DriveConstants.kFrontRightTurningMotorPort, 
         DriveConstants.kFrontRightDriveEncoderReversed, 
         DriveConstants.kFrontRightTurningEncoderReversed, 
         DriveConstants.kFrontRightDriveAbsoluteEncoderPort, 
         DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, 
         DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
     backLeft = new SwerveModule(DriveConstants.kBackLeftDriveMotorPort, 
       DriveConstants.kBackLeftTurningMotorPort, 
       DriveConstants.kBackLeftDriveEncoderReversed, 
       DriveConstants.kBackLeftTurningEncoderReversed, 
       DriveConstants.kBackLeftDriveAbsoluteEncoderPort, 
       DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, 
       DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
     backRight = new SwerveModule(DriveConstants.kBackRightDriveMotorPort, 
         DriveConstants.kBackRightTurningMotorPort, 
         DriveConstants.kBackRightDriveEncoderReversed, 
         DriveConstants.kBackRightTurningEncoderReversed, 
         DriveConstants.kBackRightDriveAbsoluteEncoderPort, 
         DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, 
         DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    //gyro
      gyro = new WPI_Pigeon2(DriveConstants.kPigeonPort);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 3001, 1000);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 3003, 1000);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 3011, 1000);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 3017, 1000);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 3021, 1000);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 3023, 1000);
      gyro.configFactoryDefault(1000);
      gyro.configZAxisGyroError(DriveConstants.kGyroZError, 1000);
      gyro.configMountPose(AxisDirection.NegativeY, AxisDirection.PositiveZ, 1000);
      zeroHeading(initialPose.getRotation().getDegrees());
      constructOdometry();
    //Dashboard
      tabSwerveSubsystem = Shuffleboard.getTab("SwerveSubsystem");
    //dashboard
      debugInit(); //initialize debug outputs
  }

  SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  SlewRateLimiter turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
  PIDController turningPID = new PIDController(DriveConstants.kPTurning, 0, DriveConstants.kDTurning);
  PIDController xPID = new PIDController(DriveConstants.kPTranslation, 0, 0);
  PIDController yPID = new PIDController(DriveConstants.kPTranslation, 0, 0);

  public void drive(double xSpeed, double ySpeed, double turningSpeed, boolean limited, boolean fieldOriented){
    SmartDashboard.putNumber("dr_xSpeed",xSpeed);
    SmartDashboard.putNumber("dr_ySpeed",ySpeed);
    SmartDashboard.putNumber("dr_rSpeed",turningSpeed);

    //  Make the driving smoother, no sudden acceleration from sudden inputs
    if(limited) {
      xSpeed = xLimiter.calculate(xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
      ySpeed = yLimiter.calculate(ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
      turningSpeed = turningLimiter.calculate(turningSpeed * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
    }
    // Construct desired chassis speeds (convert to appropriate reference frames)
    ChassisSpeeds chassisSpeeds;

    SmartDashboard.putBoolean("fieldOriented", fieldOriented);

    //3.5. Fudge Factor to eliminate uncommanded change in direction when translating and rotating simultaneously
    ySpeed += turningSpeed * (-xSpeed) * DriveConstants.kPFudge;
    //debug output: ySpeed += turningSpeed * (-xSpeed) * SmartDashboard.getNumber("kPFudge", DriveConstants.kPFudge);
    xSpeed += turningSpeed * (ySpeed - turningSpeed * (-xSpeed) * DriveConstants.kPFudge) * DriveConstants.kPFudge;
    //debug output: xSpeed += turningSpeed * ySpeed * SmartDashboard.getNumber("kPFudge", DriveConstants.kPFudge);

    // 3.55. P loops to create accurate outputs
    //turning
    //Debug intput: turningPID.setP(SmartDashboard.getNumber("kPTurning", DriveConstants.kPTurning));
    turningSpeed += turningPID.calculate(getAngularVelocity(), turningSpeed);
    //drive
      //y
        //Debug intput: yPID.setP(SmartDashboard.getNumber("kPTranslation", DriveConstants.kPTranslation));
        ySpeed += yPID.calculate(getChassisSpeeds().vyMetersPerSecond + getAngularVelocity() * getChassisSpeeds().vxMetersPerSecond * DriveConstants.kPFudge, ySpeed);
      //x
        //Debug intput: xPID.setP(SmartDashboard.getNumber("kPTranslation", DriveConstants.kPTranslation));
        xSpeed += xPID.calculate(getChassisSpeeds().vxMetersPerSecond + getAngularVelocity() * getChassisSpeeds().vyMetersPerSecond * DriveConstants.kPFudge, xSpeed);

//4 - Convert and send chasis speeds
    if(fieldOriented)
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, getRotation2d());
    else
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    setModuleStates(moduleStates);
  }
  public void toggleFieldOriented(){fieldOriented = !fieldOriented;}
  public boolean getFieldOriented(){return fieldOriented;}
  public void setFieldOriented(boolean set){fieldOriented = set;}
  public void resetPose(){
    odometry.resetPosition(new Pose2d(), new Rotation2d());
  }

  public void joystickDrive(double xSpeed, double ySpeed, double turningSpeed){
    //1.5 interpret joystick data
    //translation
    translationMagnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)); //x and y vectors to polar magnitude
    translationDirection = new Rotation2d(ySpeed, xSpeed); //creates Rotation2D representing the direction of the input vectors using yspeed/xspeed as the cos/sin of the direction
    //Rotation
    rotationMagnitude = Math.abs(turningSpeed); //magnitude of joystick input
    rotationDirection = new Rotation2d(turningSpeed, 0); //conveys polarity +/-
    //Debug output: SmartDashboard.putNumber("directionR", rotationDirection.getCos());

    //1.55 scale magnitudes to reflect deadzone
    //Translation
    translationMagnitudeScaled = (1/(1- Constants.OIConstants.kDeadband))*(translationMagnitude - Constants.OIConstants.kDeadband); //converted to be representative of deadzone using point slope form (y-y1)=(m)(x-x1) -> (scaled-minimun raw input)=(maximum input/(1-deadzone))(raw input-deadzone) -> scaled=(maximum input/(1-deadzone))(raw input-deadzone)
    //Rotation
    scaledMagnitudeRotation = (1/(1- Constants.OIConstants.kDeadband))*(rotationMagnitude- Constants.OIConstants.kDeadband); //same algorithm as scaled magnitude translation

    //2.0 deadzone and construct outputs
    //Translation
    if (translationMagnitude > Constants.OIConstants.kDeadband) {
      ySpeed = translationDirection.getCos() * translationMagnitudeScaled; //original direction, scaled magnitude... this is kinda a misnomer because this x component itself is a magnitude, but it is representative of a direction of the raw input
      xSpeed = translationDirection.getSin() * translationMagnitudeScaled;
    } else {
      xSpeed = 0.0;
      ySpeed = 0.0; //zero inputs < deadzone
    }
    //Rotation
    if (rotationMagnitude > Constants.OIConstants.kDeadband) {
      turningSpeed = rotationDirection.getCos() * scaledMagnitudeRotation; //same as above for translation
    } else {
      turningSpeed = 0.0; //zero inputs < deadzone
    }

    /*
    // 2.5 square inputs //not needed for now, only needed it when there was a bug elsewhere
    xSpeed = xSpeed * Math.abs(xSpeed);
    ySpeed = ySpeed * Math.abs(ySpeed);
    turningSpeed = turningSpeed * Math.abs(turningSpeed);
    */
    drive(xSpeed, ySpeed, turningSpeed, true, this.fieldOriented);
  }

  //Configuration
    public void zeroHeading() { //reset gyroscope to have it set the current direction as the forward direction of field when robot boots up
          gyro.zeroGyroBiasNow(1000);
          gyro.setYaw(0, 1000);
    }
  public void zeroHeading(double yaw) {
    gyro.zeroGyroBiasNow(1000);
    gyro.setYaw(yaw, 1000);
  }

    public void constructOdometry() { //constructs odometry object
      odometry = new SwerveDrivePoseEstimator(getRotation2d(), 
      initialPose, 
      DriveConstants.kDriveKinematics, 
      DriveConstants.kSDOdo, 
      DriveConstants.kSDState, 
      DriveConstants.kSDVision, 
      RobotConstants.kMainLoopPeriod);  
    }
  
  //Getters
    //A number equal to x - (y Q), where Q is the quotient of x / y rounded to the nearest integer
    //(if x / y falls halfway between two integers, the even integer is returned)
    public double getHeading() {
          return Math.IEEEremainder(gyro.getYaw(), 360); //clamps value between -/+ 180 deg where zero is forward
    }

    //module states
    public SwerveModuleState[] getStates() {
      return new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
    }

    //chassis speeds
    public ChassisSpeeds getChassisSpeeds() {
      chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(getStates());
      return fieldOriented ? 
      ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, getRotation2d()) :
      chassisSpeeds;
    }

    //get rotational velocity for closed loop
    public double getAngularVelocity() {
        return -gyro.getRate() * DriveConstants.kDegreesToRadians;
    }

    //since wpilib often wants heading in format of Rotation2d
    public Rotation2d getRotation2d() {
      return Rotation2d.fromDegrees(getHeading());
    }

  //Setters 
    public void fieldOriented(boolean isFieldOriented) {
      fieldOriented = isFieldOriented;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
      //normalizes wheel speeds in case max speed reached^^
      frontLeft.setDesiredState(desiredStates[0]);
      frontRight.setDesiredState(desiredStates[1]);
      backLeft.setDesiredState(desiredStates[2]);
      backRight.setDesiredState(desiredStates[3]);
    }

    public void stopModules() {
      frontLeft.stop();
      frontRight.stop();
      backLeft.stop();
      backRight.stop();
    }

    public void updateOdometry() {
      //odometry.updateWithTime(Timer.getFPGATimestamp(), new Rotation2d(Math.toRadians(getHeading())), getStates()); //make rotation difference work!!!
      odometry.updateWithTime(Timer.getFPGATimestamp(), new Rotation2d(Math.toRadians(getHeading())), getStates());

      //debug output: SmartDashboard.putNumber("OdoH", odometry.getEstimatedPosition().getRotation().getDegrees());
      //debug output: SmartDashboard.putNumber("gyroH", getHeading());
      //debug output: SmartDashboard.putNumber("OdoY", Units.metersToFeet(odometry.getEstimatedPosition().getY()));
      //debug output: SmartDashboard.putNumber("OdoX", Units.metersToFeet(odometry.getEstimatedPosition().getX()));      
    }
    
    //Dashboard
    //Debugging
      //Odometry
        public void debugOdometryInit () {
          //Odometry
            dashboardOdometryHeading = tabSwerveSubsystem.add("OdometryHeading", odometry.getEstimatedPosition().getRotation().getDegrees()).getEntry();
            dashboardOdometryY = tabSwerveSubsystem.add("OdometryY", odometry.getEstimatedPosition().getY()).getEntry();
            dashboardOdometryX = tabSwerveSubsystem.add("OdometryX", odometry.getEstimatedPosition().getX()).getEntry();
        }
        public void debugOdometryPeriodic () {
          //Odometry
            dashboardOdometryHeading.setDouble(odometry.getEstimatedPosition().getRotation().getDegrees());
            dashboardOdometryY.setDouble(odometry.getEstimatedPosition().getY());
            dashboardOdometryX.setDouble(odometry.getEstimatedPosition().getX());
        }
      //Other
        public void debugInit() {
          //Odometry
            debugOdometryInit();
        }
        public void debugPeriodic() {
          //Chassis Speeds
            //debug output: tabSwerveSubsystem.add("CSH", getChassisSpeeds().omegaRadiansPerSecond);
            //debug output: tabSwerveSubsystem.add("CSY", getChassisSpeeds().vyMetersPerSecond);
            //debug output: tabSwerveSubsystem.add("CSX", getChassisSpeeds().vxMetersPerSecond);
          //Odometry
            debugOdometryPeriodic();
          //Gryo
            //debug output: tabSwerveSubsystem.add("gyroH", getHeading());
        }

  //Periodic
    @Override
    public void periodic() {
      //update odometry
        updateOdometry();
      //dashboard outputs
        debugPeriodic();
    }



    // Vision stuff

  public void fieldTagSpotted(FieldTag fieldTag, Transform3d transform, double latency){

  //1. calculate X and Y position of camera based on X and Y components of tag and create a pose from that
    Rotation2d newRotation = new Rotation2d( ( Math.IEEEremainder((-transform.getRotation().getZ() - fieldTag.getPose().getRotation().getRadians()+4*Math.PI),2*Math.PI)));
    double newY = 0-( transform.getY()* Math.cos(-newRotation.getRadians()) + transform.getX() * Math.cos(Math.PI/2 - newRotation.getRadians()) + fieldTag.getPose().getY() ) ;
    double newX = 0- ( transform.getY()*Math.sin(-newRotation.getRadians()) + transform.getX() * Math.sin(Math.PI/2 - newRotation.getRadians()) + fieldTag.getPose().getX() ) ;
    Pose2d newPose = new Pose2d(newX,newY, newRotation);

    SmartDashboard.putNumber("new_x", newX);
    SmartDashboard.putNumber("new_y", newY);

    //2. Pass vision measurement to odometry
    SmartDashboard.putNumber("new rotation",newRotation.getDegrees());
    odometry.addVisionMeasurement(new Pose2d(newX,newY,newRotation),Timer.getFPGATimestamp() - latency*(1.0/1000));
  //  zeroHeading(newRotation.getDegrees());

  }

  public double[] getDesiredSpeeds(Pose2d pose){
    //get desired X and Y speed to reach a given pose
    double[] out = new double[3];
    double rotationDiff = (pose.getRotation().getRadians() - odometry.getEstimatedPosition().getRotation().getRadians());
    double xDiff = (pose.getX() - odometry.getEstimatedPosition().getX());
    double yDiff = (pose.getY() - odometry.getEstimatedPosition().getY());
    double dist = Math.sqrt(Math.pow(xDiff,2) + Math.pow(yDiff,2));

    double dirToPose = Math.atan2(yDiff,xDiff);

    out[0] = dist * Math.cos(dirToPose +rotationDiff) * 0.5;
    out[1] = dist * Math.sin(dirToPose+rotationDiff) * 0.5;
    out[2] = rotationDiff * 0.7;

    if(Math.abs(rotationDiff)<Math.toRadians(2)) out[2] = 0;
    if(Math.abs(out[0])<0.005) out[0] = 0;
    if(Math.abs(out[1])<0.005) out[1] = 0;

    return out;
  }

  public void driveToPose(Pose2d pose){
    double[] speeds = getDesiredSpeeds(pose);

    speeds[0] = MathThings.absMax(speeds[0],0.2);
    speeds[1] = MathThings.absMax(speeds[1],0.2);
    speeds[2] = MathThings.absMax(speeds[2],0.2);

    drive(speeds[0],speeds[1],speeds[2],true,false);
  }



}