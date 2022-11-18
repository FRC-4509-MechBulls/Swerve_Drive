// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  //motors
  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turningMotor;

  //abs encoder
  private final DutyCycleEncoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  //PID
  private final PIDController turningPidController;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
          int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    //absolute encoder
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);
    absoluteEncoder.setDistancePerRotation(1);
    
    //motors
    driveMotor = new WPI_TalonFX(driveMotorId);
    turningMotor = new WPI_TalonFX(turningMotorId);
    driveMotor.configFactoryDefault();
    turningMotor.configFactoryDefault();
    turningMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.setNeutralMode(NeutralMode.Coast);
    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);
    turningMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    
    //initialize pid controller
    turningPidController = new  PIDController(ModuleConstants.kPTurning, 0, 0); //proportional control is enough
    turningPidController.enableContinuousInput(-Math.PI, Math.PI); //tells PID that system is circular

    //initialize encoders in thread so they don't timeout
    new Thread(() -> {
      try {
              Thread.sleep(1000);
              resetEncoders();
      } catch (Exception e) {
      }

}).start();

}

  //is this even used?
  public double getDrivePosition() {
    return driveMotor.getSelectedSensorPosition();
  }

  public double getTurningPosition() {
    return turningMotor.getSelectedSensorPosition() / ModuleConstants.kRadiansToTurning;
  }

  public double getDriveVelocity() {
    return driveMotor.getSelectedSensorVelocity() / ModuleConstants.kMetersToDrive; //convert raw sensor units to m/s
  }

  //is this even used?
  public double getTurningVelocity() {
    return turningMotor.getSelectedSensorVelocity();
  }

  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getAbsolutePosition(); //range 0-1
    angle *= ModuleConstants.kAbsToRadians; //converts to radians
    angle += absoluteEncoderOffsetRad; //subtracts the offset to get the actual wheel angles
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0); //multiply -1 if reversed
  }

  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0); //reset drive motor encoder to 0
    turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad() * ModuleConstants.kRadiansToTurning); //resets turning motor encoder to absolute encoder value
  }

  //wpi lib requests info in form of swerve module state, so this method converts it
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    //prevents wheels from going to OG pos when joysticks are not moved
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    //Debug output: SmartDashboard.putNumber("preOpRadians" + absoluteEncoder.getSourceChannel(), state.angle.getRadians());
    state = SwerveModuleState.optimize(state, getState().angle); //makes it so wheel never turns more than 90 deg

    driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond); //scales vel down using max speed
    turningMotor.set(TalonFXControlMode.PercentOutput, turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    //^^^calculates output for the angle setpoint and current pos
    //Debug output: SmartDashboard.putString("Swerve[" + absoluteEncoder.getSourceChannel() + "] state", state.toString()); //debugging info
    //Debug output: SmartDashboard.putNumber("setRadians" + absoluteEncoder.getSourceChannel(), state.angle.getRadians());
  }

  //stops both motors on the module
  public void stop() {
    driveMotor.set(TalonFXControlMode.PercentOutput, 0);
    turningMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Debug output: SmartDashboard.putBoolean("absPos"+this.turningMotor.getDeviceID(), absoluteEncoder.isConnected());
    //Debug output: SmartDashboard.putNumber("relRadians" + absoluteEncoder.getSourceChannel(), getTurningPosition());
    //Debug output: SmartDashboard.putNumber("absRadians" + absoluteEncoder.getSourceChannel(), getAbsoluteEncoderRad());
    //Debug output: SmartDashboard.putNumber("abs0-1" + absoluteEncoder.getSourceChannel(), absoluteEncoder.getAbsolutePosition());
    //Debug output: SmartDashboard.putNumber(this.name+".sDrivePos",getDrivePosition());
  }
}
