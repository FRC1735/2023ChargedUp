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

public class Claw extends SubsystemBase {
  CANSparkMax motor;
  
  private final double SPEED = 1;
  private final double PID_SPEED = 1;

  private SparkMaxAbsoluteEncoder absoluteEncoder;
  private SparkMaxPIDController pidController;
  
  /** Creates a new Claw. */
  public Claw() {
    motor = new CANSparkMax(Constants.ClawConstants.canId, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);

    absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder.setInverted(true);
  
    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(absoluteEncoder);

    pidController.setP(2);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setFF(0);
    pidController.setOutputRange(-PID_SPEED, PID_SPEED);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Claw Encoder", absoluteEncoder.getPosition());
    //SmartDashboard.putNumber("Claw zero offset", absoluteEncoder.getZeroOffset());
    //SmartDashboard.putNumber("Claw Abs Velocity", absoluteEncoder.getVelocity());
    //SmartDashboard.putNumber("Claw Abs applied output", motor.getAppliedOutput());
  }

  public void manualOpen() {
    motor.set(SPEED);
  }

  public void manualClose() {
    motor.set(-SPEED);
  }

  public void stop() {
    motor.stopMotor();
  }

  public void open() {
    pidController.setReference(0.80, ControlType.kPosition);
  }

  public void cone() {
    pidController.setReference(0.56, ControlType.kPosition);
  }

  public void cube() {
    pidController.setReference(0.56, ControlType.kPosition);
  }

  public void setToZero() {
    motor.set(0);
  }
}
