// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
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

  //Values
	private double delta = 0;
	private double deltaConverted = 0;
	private double setAngle = 0;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    //absolute encoder
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);
    absoluteEncoder.setDistancePerRotation(1);
    
    //motors
      //drive
      driveMotor = new WPI_TalonFX(driveMotorId);
      driveMotor.configFactoryDefault();
      driveMotor.setNeutralMode(NeutralMode.Coast);
      driveMotor.setInverted(driveMotorReversed);
			driveMotor.configVoltageCompSaturation(12);
      //turn
      turningMotor = new WPI_TalonFX(turningMotorId);
      turningMotor.configFactoryDefault();
      turningMotor.setNeutralMode(NeutralMode.Coast);
      turningMotor.setInverted(turningMotorReversed);
      turningMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
      turningMotor.config_kP(0, ModuleConstants.kPTurning);
      turningMotor.config_kD(0, ModuleConstants.kDTurning);
			turningMotor.configVoltageCompSaturation(12);
      //both
      enableVoltageCompensation(true);
      //initialize encoders in thread so they don't timeout
      new Thread(() -> {
        try {
              Thread.sleep(1000);
              resetEncoders();
        } catch (Exception e) {}
      }).start();
		
		//Values
		//Debug output: SmartDashboard.putNumber("kPT", ModuleConstants.kPTurning);
    //Debug output: SmartDashboard.putNumber("kDT", ModuleConstants.kDTurning);
  }

  //Configuration
  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0); //reset drive motor encoder to 0
    turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad() * ModuleConstants.kRadiansToTurning); //resets turning motor encoder to absolute encoder value
  }
  public void enableVoltageCompensation(boolean onOff) {
    driveMotor.enableVoltageCompensation(onOff);
    turningMotor.enableVoltageCompensation(onOff);
  } 

  //Getters
  public double getTurningPosition() {
    return turningMotor.getSelectedSensorPosition() / ModuleConstants.kRadiansToTurning;
  }
  public double getDriveVelocity() {
    return driveMotor.getSelectedSensorVelocity() / ModuleConstants.kMetersToDrive; //convert raw sensor units to m/s
  }
  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getAbsolutePosition(); //range 0-1
    angle *= ModuleConstants.kAbsToRadians; //converts to radians
    angle += absoluteEncoderOffsetRad; //subtracts the offset to get the actual wheel angles
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0); //multiply -1 if reversed
  }
  public SwerveModuleState getState() { //wpi lib requests info in form of swerve module state, so this method converts it
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  //Setters
  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) { //prevents wheels from going to OG pos when joysticks are not moved
      stop();
      return;
    }
    //Debug output: SmartDashboard.putNumber("preOpRadians" + absoluteEncoder.getSourceChannel(), state.angle.getRadians());
    state = SwerveModuleState.optimize(state, getState().angle); //makes it so wheel never turns more than 90 deg
		delta = state.angle.getRadians() - getTurningPosition(); //error
		deltaConverted = delta % Math.PI; //error converted to representative of the actual gap; error > pi indicates we aren't taking the shortest route to setpoint, but rather doing one or more 180* rotations.this is caused by the discontinuity of numbers(pi is the same location as -pi, yet -pi is less than pi)
		setAngle = Math.abs(deltaConverted) < (Math.PI / 2) ? getTurningPosition() + deltaConverted : getTurningPosition() - ((Math.abs(deltaConverted) * (Math.PI/deltaConverted)) * (1-Math.abs(deltaConverted/Math.PI))); //makes set angle +/- 1/2pi of our current position(capable of pointing all directions)

    driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond); //scales vel down using max speed
    turningMotor.set(TalonFXControlMode.Position, setAngle * ModuleConstants.kRadiansToTurning);//, DemandType.ArbitraryFeedForward, arbitraryFF); //Position Control w/ Arbitrary Feedforward
    
    //Debug output: SmartDashboard.putNumber("stateAngle" + absoluteEncoder.getSourceChannel(), getState().angle.getRadians());
    //Debug output: SmartDashboard.putString("Swerve[" + absoluteEncoder.getSourceChannel() + "] state", state.toString());
    //Debug output: SmartDashboard.putNumber("setRadians" + absoluteEncoder.getSourceChannel(), state.angle.getRadians());
    //Debug output: SmartDashboard.putNumber("setT" + absoluteEncoder.getSourceChannel(), turningMotor.getClosedLoopTarget());
    //Debug output: SmartDashboard.putNumber("voltageT" + absoluteEncoder.getSourceChannel(), turningMotor.getMotorOutputVoltage());
  }
  public void stop() { //sets both voltage outputs to 0
    driveMotor.set(TalonFXControlMode.PercentOutput, 0);
    turningMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
		//Debug input: turningMotor.config_kP(0, SmartDashboard.getNumber("kPT", ModuleConstants.kPTurning));
    //Debug input: turningMotor.config_kD(0, SmartDashboard.getNumber("kDT", ModuleConstants.kDTurning));
		//Debug output: SmartDashboard.putBoolean("absPos"+this.turningMotor.getDeviceID(), absoluteEncoder.isConnected());
    //Debug output: SmartDashboard.putNumber("relRadians" + absoluteEncoder.getSourceChannel(), getTurningPosition() * ModuleConstants.kRadiansToTurning);
    //Debug output: SmartDashboard.putNumber("absRadians" + absoluteEncoder.getSourceChannel(), getAbsoluteEncoderRad());
    //Debug output: SmartDashboard.putNumber("abs0-1" + absoluteEncoder.getSourceChannel(), absoluteEncoder.getAbsolutePosition());
    //Debug output: SmartDashboard.putNumber(this.name+".sDrivePos",getDrivePosition());
    //Debug output: SmartDashboard.putNumber("errorT" + absoluteEncoder.getSourceChannel(), turningMotor.getClosedLoopError());
    //Debug output: SmartDashboard.putNumber("deltaC" + absoluteEncoder.getSourceChannel(), deltaConverted);
  }
}
