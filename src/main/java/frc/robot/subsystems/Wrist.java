// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  private CANSparkMax motor;
  private final double SPEED = 1;
  private final double PID_SPEED = 0.8;
  private SparkMaxAbsoluteEncoder absoluteEncoder;
  private SparkMaxPIDController pidController;

  /** Creates a new Wrist. */
  public Wrist() {
    motor = new CANSparkMax(Constants.WristConstants.canId, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);

    absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder.setInverted(true);
    absoluteEncoder.setZeroOffset(0.5);

    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(absoluteEncoder);

    pidController.setP(2);
    pidController.setI(0.0);
    pidController.setD(0);
    pidController.setFF(0);
    pidController.setOutputRange(-PID_SPEED, PID_SPEED);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Encoder", absoluteEncoder.getPosition());
    //SmartDashboard.putNumber("Wrist zero offset", absoluteEncoder.getZeroOffset());
    //SmartDashboard.putNumber("Wrist Abs Velocity", absoluteEncoder.getVelocity());
    //SmartDashboard.putNumber("Wrist Abs applied output", motor.getAppliedOutput());
  }

  public void up() {
    // todo - limits
    motor.set(-SPEED);
  }

  public void down() {
    // todo - limits
    motor.set(SPEED);
  }

  public void stop() {
    motor.stopMotor();
  }

  public void storage() {
    pidController.setReference(0.34, CANSparkMax.ControlType.kPosition); 
  }

  public void scoreMid() {
    pidController.setReference(0.81, CANSparkMax.ControlType.kPosition);
  }

  public void pickupFront() {
    pidController.setReference(0.69, CANSparkMax.ControlType.kPosition);
  }

  public void scoreHigh() {
    pidController.setReference(0.75, CANSparkMax.ControlType.kPosition);
  }

  public void pickupAbove() {
    pidController.setReference(0.94, CANSparkMax.ControlType.kPosition);
  }

  public void humanPlayerStation() {
    pidController.setReference(0.88, CANSparkMax.ControlType.kPosition);
  }

  public void setToZero() {
    motor.set(0);
  }
}
