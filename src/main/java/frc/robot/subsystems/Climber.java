// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class Climber extends SubsystemBase {
  CANSparkMax leftClimber;
  CANSparkMax rightClimber;

  public Climber() {
    leftClimber = new CANSparkMax(SubsystemConstants.leftClimber, MotorType.kBrushless);
    rightClimber = new CANSparkMax(SubsystemConstants.rightClimber, MotorType.kBrushless);
  }

  public void climb(double speed) {
    leftClimber.set(speed);
    rightClimber.follow(leftClimber,true);
  }
}
