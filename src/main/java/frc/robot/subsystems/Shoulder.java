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
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShoulderConstants;

public class Shoulder extends SubsystemBase {
  CANSparkMax leftMotor;
  CANSparkMax rightMotor;

  private final double SPEED = 0.5;
  private SparkMaxAbsoluteEncoder absoluteEncoder;
  private SparkMaxPIDController pidController;


  /** Creates a new Shoulder. */
  public Shoulder() {
    leftMotor = new CANSparkMax(ShoulderConstants.leftCanId, MotorType.kBrushless);
    rightMotor = new CANSparkMax(ShoulderConstants.rightCanId, MotorType.kBrushless);

    leftMotor.setIdleMode(IdleMode.kBrake);
    absoluteEncoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder.setInverted(false);
    pidController = leftMotor.getPIDController();
    pidController.setFeedbackDevice(absoluteEncoder);

    pidController.setP(0.6);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setFF(0);
    pidController.setOutputRange(-1, 1);

    //pidController.setPositionPIDWrappingEnabled(true);


    rightMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.follow(leftMotor, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shoulder Encoder", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("zero offset", absoluteEncoder.getZeroOffset());

    SmartDashboard.putNumber("Abs Velocity", absoluteEncoder.getVelocity());
    SmartDashboard.putNumber("Abs applied output", leftMotor.getAppliedOutput());
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

  public void setPosition() {
    pidController.setReference(0.5, CANSparkMax.ControlType.kPosition);
  }
}
