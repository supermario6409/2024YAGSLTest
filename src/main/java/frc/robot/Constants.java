// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double robotMass = 31.751; //mass in kilograms
  public static final Matter chassis = new Matter(new Translation3d(0,0,Units.inchesToMeters(8)), robotMass);
  public static final double loopTime = 0.13; //20ms for the regular loop time + 110 ms for the spark max delay.

  public static final class AutoConstants {
    public static final PIDConstants translationPID = new PIDConstants(0.7,0,0);
    public static final PIDConstants anglePID = new PIDConstants(0.4,0,0.01);
  }

  public static final class DrivebaseConstants {
    //time motor spends paused in seconds on brake activation
    public static final double brakeTime = 2;
    //speed modifier
    public static final double speedMod = 0.2;
  }

  public static final class SubsystemConstants {
    public static final int leftClimber = 13;
    public static final int rightClimber = 14;
    public static final double climberSpeed = 0.2;
  }

  public static final class OperatorConstants {
    //joystick deadband
    public static final double leftXDeadzone = 0.1;
    public static final double leftYDeadzone = 0.1;
    public static final double rightXDeadzone = 0.1;
    public static final double rightYDeadzone = 0.1;
  }
}

