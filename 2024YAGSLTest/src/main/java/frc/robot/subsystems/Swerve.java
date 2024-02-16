// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.trajectory.Trajectory;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;

public class Swerve extends SubsystemBase {
  //create a swervedrive object using YAGSL
  private final SwerveDrive swerveDrive;
  //define the max speed of the robot in meters per second to limit acceleration
  public double maximumSpeed = Units.feetToMeters(14.5);
  //config the json files
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
 
  public Swerve(File directory) {
    //angle conversion factor is 360/(gear ratio * encoder resolution)
    //so for our module this year it is 12.8 motor revolutions per wheel rotation
    //a neo has an encoder resolution of 42 ticks per rotation.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
    //Motor conversion factor is 6.75 motor rotations per wheel rotation
    //encoder again is 42 ticks per rotation
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);
    //configure the Telemetry before making the swervedrive to avoid redundant object generation
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try{
      //works because we're supplying the conversion factor with a JSON file
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); //I don't think we're correcting with just angles
    swerveDrive.updateCacheValidityPeriods(0, 0, 0); //read the docs.
  }

  //construct the swervedrive
  public Swerve(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive((driveCfg), controllerCfg, maximumSpeed);
  }
//set up autobuilder for pathplanner(build paths on the fly)
public void setupPathPlanner() {
  AutoBuilder.configureHolonomic(
    this::getPose, //robot pose supplier
    this::resetOdometry,  //reset the odometry(called if auto has a starting pose)
    this::getRobotVelocity, //chassis speeds supplier(relative, not absolute)
    this::setChassisSpeeds, //method that will drive the robot given relative chassisspeeds
    new HolonomicPathFollowerConfig(//set this stuff up in constants
      null, //translation PID values
      null, //rotation PID values
      maximumSpeed, //max module speed in m/s
      swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(), // drive base radius in meters. distance from center of bot to furthest module 
      new ReplanningConfig()), //default path planning config
     () -> { //a set of code to mirror the path if we are on the red alliance.
      var alliance = DriverStation.getAlliance();
      return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
     }, 
     this); //reference to this subsystem to set requirements.
}

//setup the autonomous path follower
public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
  PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

  if (setOdomToStart) {
    resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
  }

  //create a path following the command using the autobuilder
  return AutoBuilder.followPath(path);
}

//use pathplanner path finding to go to a point on the field
public Command driveToPose(Pose2d pose) {
  PathConstraints constraints = new PathConstraints(
    swerveDrive.getMaximumVelocity(), 
    4.0,
    swerveDrive.getMaximumAngularVelocity(), 
    Units.degreesToRadians(720));
    //since we configured the autobuilder above, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
      pose, 
      constraints,
      0.0,
      0.0);
}

public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
  return run(() -> {
    double xInput = Math.pow(translationX.getAsDouble(), 3);//smooth control out
    double yInput = Math.pow(translationY.getAsDouble(),3) ;
    //make the robot actually move
    driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, 
    headingX.getAsDouble(),
    headingY.getAsDouble(), 
    swerveDrive.getOdometryHeading().getRadians(),
    swerveDrive.getMaximumVelocity()));
  });
}

//command to characterize the robot drive motors using SysId
public Command sysIdDriveMotorCommand() {
  return SwerveDriveTest.generateSysIdCommand(
    SwerveDriveTest.setDriveSysIdRoutine(
      new Config(), 
      this, swerveDrive, 12), 
      3.0, 5.0, 3.0);
}

//command to characterize the robot angle motors using SysId
public Command sysIdAngleMotorCommand() {
  return SwerveDriveTest.generateSysIdCommand(
    SwerveDriveTest.setAngleSysIdRoutine(
      new Config(), 
      this, swerveDrive), 
      3.0, 5.0, 3.0);
}

//command to drive the robot using translative values and heading as angular velocity.
public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
  return run(() -> {
    //make the robot move
    swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
                                        Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
                      Math.pow(angularRotationX.getAsDouble(),3) * swerveDrive.getMaximumAngularVelocity(),
                      true,
                      false);
  });
}

//primary method for controlling the drivebase(basically the others are nice to have in case things go wrong)
//this method is akin to the 2023 Atlas code, where a translation2d state and a rotaiton state are given and the modules are commanded accordingly.

public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
  swerveDrive.drive(
  translation,
  rotation,
  fieldRelative,
  false);
}

//drive the robot given a chassis field oriented velocity
public void driveFieldOriented(ChassisSpeeds velocity) {
  swerveDrive.driveFieldOriented(velocity);
}

//drive the robot given a chassis robot oriented velocity
public void drive(ChassisSpeeds velocity) {
  swerveDrive.drive(velocity); 
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //get the swerve drive kinematics object.
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  //reset odometry to the given pose, required to run if gyro angle or module positions are reset
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry((initialHolonomicPose));
  }

  //grab the current pose (position and rotation) of the robot, as reported by the odometry
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  //set the chassis speeds with closed-loop velocity control
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  //post the trajectory to the field
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  //reste the gyro angle to zero and resets the odometry to the same position, but facing towards zero
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  //set the drive motors to brake/coast mode
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  //get the current yaw angle of the robot, as reported by the swerve pose estimator
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }
  
  //get the chassis speeds based on controller input of 2 joysticks, one for speeds in a direction, the other for the angle of the robot.
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(
      xInput, 
      yInput, 
      headingX, 
      headingY, 
      getHeading().getRadians(),
      maximumSpeed);
  }

  //get the chassis speeds based on controller input of 1 joystick and one angle, control the robot at an offset of 90 degrees
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(
      xInput, 
      yInput, 
      angle.getRadians(), 
      getHeading().getRadians(), 
      maximumSpeed);
  }

  //gets the current field-relative velocity of the robot
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  //gets the current velocity of the robot(relative)
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  //get the swerve controller in the swerve drive
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  //get the swervedriveconfiguration object
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  //lock the swerve drive to prevetn it from moving(not the same thing as the balance lock)
  public void lock() {
    swerveDrive.lockPose();
  }

  //gets the current pitch angle of the robot, as reported by the imu
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

}
