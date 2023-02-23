// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  private CANSparkMax motor;
  private final double SPEED = 0.1;
  private final double PID_SPEED = 0.1;
  private SparkMaxAbsoluteEncoder absoluteEncoder;
  private SparkMaxPIDController pidController;

  /** Creates a new Wrist. */
  public Wrist() {
    motor = new CANSparkMax(Constants.WristConstants.canId, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);

    absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    // ???
    // absoluteEncoder.setInverted(false);
    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(absoluteEncoder);

    pidController.setP(0);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setFF(0);
    pidController.setOutputRange(-PID_SPEED, PID_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Wrist Encoder", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Wrist zero offset", absoluteEncoder.getZeroOffset());

    SmartDashboard.putNumber("Wrist Abs Velocity", absoluteEncoder.getVelocity());
    SmartDashboard.putNumber("Wrist Abs applied output", motor.getAppliedOutput());
  
  }

  public void up() {
    motor.set(-SPEED);
  }

  public void down() {
    motor.set(SPEED);
  }

  public void stop() {
    motor.stopMotor();
  }

  // todo: name
  public void open() {
    pidController.setReference(0.5, CANSparkMax.ControlType.kPosition);
  }

  public void setToZero() {
    motor.set(0);
  }
}
