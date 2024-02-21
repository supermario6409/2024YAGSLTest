// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class teleDrive extends Command {
  private final Swerve swerve;
  private final DoubleSupplier vX, vY;
private final DoubleSupplier heading;
private boolean initRotation = false;

  public teleDrive(Swerve swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;

    addRequirements(swerve);
  }
    //TODO Auto-generated constructor stub

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initRotation = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //get the desired chassis speeds based on a 2 joystick module(one to drive, one to turn)
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), new Rotation2d(heading.getAsDouble() * Math.PI));

    // limit velocity to prevent the robot from deciding to pull a pitt pirates
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(
      translation, 
      swerve.getFieldVelocity(), 
      swerve.getPose(), 
      Constants.loopTime, 
      Constants.robotMass, 
      List.of(Constants.chassis), 
      swerve.getSwerveDriveConfiguration());
      //put info to the smartDashboard
      SmartDashboard.putNumber("LimitedTranslation", translation.getX());
      SmartDashboard.putString("Translation", translation.toString());

      //actually make it move after all that math
      swerve.drive(translation,desiredSpeeds.omegaRadiansPerSecond,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
