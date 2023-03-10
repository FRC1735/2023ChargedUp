// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  CANSparkMax motor;

  double SPEED = 0.2; //1;
  double PID_SPEED = 1;

  double OUT_LIMIT = 0.08;
  double IN_LIMIT = 0.933;

  private SparkMaxAbsoluteEncoder absoluteEncoder;
  private SparkMaxPIDController pidController;

  /** Creates a new Arm. */
  public Arm() {
    this.motor = new CANSparkMax(ArmConstants.canId, MotorType.kBrushless);
    absoluteEncoder = this.motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    pidController = this.motor.getPIDController();
    pidController.setFeedbackDevice(absoluteEncoder);

    pidController.setP(1.5);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setFF(0);
    pidController.setOutputRange(-PID_SPEED, PID_SPEED);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm encoder", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("arm abs velocity", absoluteEncoder.getVelocity());
    SmartDashboard.putNumber("arm applied output", motor.getAppliedOutput());
  }

  public void in() {
    if (absoluteEncoder.getPosition() < IN_LIMIT) {
      motor.set(SPEED);
    }
  }

  public void out() {
    if (absoluteEncoder.getPosition() > OUT_LIMIT ) {
      motor.set(-SPEED);
    }
  }

  public void storage() {
    pidController.setReference(0.94, CANSparkMax.ControlType.kPosition); 
  }

  public void scoreMid() {
    pidController.setReference(0.71, CANSparkMax.ControlType.kPosition);
  }

  public void pickupFront() {
    pidController.setReference(0.62, CANSparkMax.ControlType.kPosition);
  }

  public void scoreHigh() {
    pidController.setReference(0.04, CANSparkMax.ControlType.kPosition);
  }

  public void pickupAbove() {
    pidController.setReference(0.49, CANSparkMax.ControlType.kPosition);
  }

  public void humanPlayerStation() {
    pidController.setReference(0.96, CANSparkMax.ControlType.kPosition);
  }

  public void stop() {
    motor.stopMotor();
  }

  public void setToZero() {
    this.motor.set(0);
  }
}
