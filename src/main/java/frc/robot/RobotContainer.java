// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);

  private final Command rc_drive = new RunCommand(()-> swerveSubsystem.joystickDrive(driverController.getLeftY()*-1,driverController.getLeftX()*-1,driverController.getRightX()*-1), swerveSubsystem);

    
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
    //   swerveSubsystem,
    //   () -> -driverController.getRawAxis(OIConstants.kDriverYAxis),
    //   () -> driverController.getRawAxis(OIConstants.kDriverXAxis),
    //   () -> driverController.getRawAxis(OIConstants.kDriverRotAxis),
    //   () -> driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
    // ));
    swerveSubsystem.setDefaultCommand(rc_drive);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, XboxController.Button.kStart.value).whenPressed(() -> swerveSubsystem.zeroHeading());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
