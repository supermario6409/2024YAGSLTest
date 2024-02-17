// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.teleDrive;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Swerve drivebase = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //CommandJoystick driver = new CommandJoystick(0);
  XboxController driver = new XboxController(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    teleDrive closedTeleDrive = new teleDrive(
    drivebase, 
    () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.leftYDeadzone), 
    () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.leftXDeadzone), 
    () -> MathUtil.applyDeadband(driver.getRightX(), OperatorConstants.rightXDeadzone));
   
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.leftYDeadzone), 
    () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.leftXDeadzone), 
    () -> MathUtil.applyDeadband(driver.getRightX(), OperatorConstants.rightXDeadzone));

    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  //apply deadbands and invert controls because joysticks are back-right positive whereas robots are front-left positive.
  //left stick controls translation
  //right stick controls angular velocity(turn speed)
  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(driver, XboxController.Button.kY.value).onTrue(new InstantCommand(drivebase::zeroGyro));
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
