// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;

public class Shoulder extends SubsystemBase {
  CANSparkMax leftMotor;
  CANSparkMax rightMotor;

  private final double SPEED = 0.5;
  private SparkMaxAbsoluteEncoder absoluteEncoder;

  /** Creates a new Shoulder. */
  public Shoulder() {
    leftMotor = new CANSparkMax(ShoulderConstants.leftCanId, MotorType.kBrushless);
    rightMotor = new CANSparkMax(ShoulderConstants.rightCanId, MotorType.kBrushless);

    leftMotor.setIdleMode(IdleMode.kBrake);
    absoluteEncoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);

    rightMotor.setInverted(true);
    rightMotor.follow(leftMotor);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shoulder Encoder", absoluteEncoder.getPosition());
  }

  public void up() {
    leftMotor.set(SPEED);
  }

  public void down() {
    leftMotor.set(-SPEED);
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
