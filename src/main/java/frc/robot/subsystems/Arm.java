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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.utils.Utils;

public class Arm extends SubsystemBase {
  CANSparkMax motor;

  double SPEED = 0.2; //1;
  double PID_SPEED = 1;

  double OUT_LIMIT = 0.08;
  double IN_LIMIT = 0.933;

  double SETPOINT_STORAGE = 0.95;
  double SETPOINT_SCORE_MID = 0.95;
  double SETPOINT_PICKUP_FRONT = 0.94;
  double SETPOINT_SCORE_HIGH = 0.28;
  double SETPOINT_PICKUP_ABOVE = 0.49;
  double SETPOINT_HUMAN_PLAYER_STATION = 0.96;

  private SparkMaxAbsoluteEncoder absoluteEncoder;
  private SparkMaxPIDController pidController;

  /** Creates a new Arm. */
  public Arm() {
    this.motor = new CANSparkMax(ArmConstants.canId, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
    absoluteEncoder = this.motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    pidController = this.motor.getPIDController();
    pidController.setFeedbackDevice(absoluteEncoder);
    pidController.setPositionPIDWrappingEnabled(false);

    pidController.setP(1.5);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setFF(0);
    pidController.setOutputRange(-PID_SPEED, PID_SPEED);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder", absoluteEncoder.getPosition());
    //SmartDashboard.putNumber("arm abs velocity", absoluteEncoder.getVelocity());
    //SmartDashboard.putNumber("arm applied output", motor.getAppliedOutput());
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
    pidController.setReference(SETPOINT_STORAGE, CANSparkMax.ControlType.kPosition); 
  }

  public boolean isAtStorage() {
    return Utils.isCloseEnough(absoluteEncoder.getPosition(), SETPOINT_STORAGE);
  }

  public void scoreMid() {
    pidController.setReference(SETPOINT_SCORE_MID, CANSparkMax.ControlType.kPosition);
  }

  public boolean isAtScoreMid() {
    return Utils.isCloseEnough(absoluteEncoder.getPosition(), SETPOINT_SCORE_MID);
  }

  public void pickupFront() {
    pidController.setReference(SETPOINT_PICKUP_FRONT, CANSparkMax.ControlType.kPosition);
  }

  public boolean isAtPickupFront() {
    return Utils.isCloseEnough(absoluteEncoder.getPosition(), SETPOINT_PICKUP_FRONT);
  }

  public void scoreHigh() {
    pidController.setReference(SETPOINT_SCORE_HIGH, CANSparkMax.ControlType.kPosition);
  }

  public boolean isAtScoreHigh() {
    return Utils.isCloseEnough(absoluteEncoder.getPosition(), SETPOINT_SCORE_HIGH);
  }

  public void pickupAbove() {
    pidController.setReference(SETPOINT_PICKUP_ABOVE, CANSparkMax.ControlType.kPosition);
  }

  public boolean isAtPickupAbove() {
    return Utils.isCloseEnough(absoluteEncoder.getPosition(), SETPOINT_PICKUP_ABOVE);
  }

  public void humanPlayerStation() {
    pidController.setReference(SETPOINT_HUMAN_PLAYER_STATION, CANSparkMax.ControlType.kPosition);
  }

  public boolean isAtHumanPlayerStation() {
    return Utils.isCloseEnough(absoluteEncoder.getPosition(), SETPOINT_HUMAN_PLAYER_STATION);
  }

  public void stop() {
    motor.stopMotor();
  }

  public void setToZero() {
    this.motor.set(0);
  }
}
