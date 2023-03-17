// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;
import frc.utils.Utils;

public class Shoulder extends SubsystemBase {
  CANSparkMax leftMotor;
  CANSparkMax rightMotor;

  private final double SPEED = 0.1;
  private final double PID_SPEED = 0.2;
  private SparkMaxAbsoluteEncoder absoluteEncoder;
  private SparkMaxPIDController pidController;

  public double SETPOINT_STORAGE = 0.96;
  public double SETPOINT_SCORE_MID = 0.24;
  public double SETPOINT_PICKUP_FRONT = 0.79;
  public double SETPOINT_SCORE_HIGH = 0.2;
  public double SETPOINT_PICKUP_ABOVE = 0.7;
  public double SETPOINT_HUMAN_PLAYER_STATION = 0.24;


  /** Creates a new Shoulder. */
  public Shoulder() {
    leftMotor = new CANSparkMax(ShoulderConstants.leftCanId, MotorType.kBrushless);
    rightMotor = new CANSparkMax(ShoulderConstants.rightCanId, MotorType.kBrushless);

    leftMotor.setIdleMode(IdleMode.kBrake);
    absoluteEncoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder.setInverted(false);
    absoluteEncoder.setZeroOffset(.6);
    pidController = leftMotor.getPIDController();
    pidController.setFeedbackDevice(absoluteEncoder);

    pidController.setP(1.2);
    pidController.setI(0);
    pidController.setD(0.2);
    pidController.setFF(0);
    pidController.setOutputRange(-PID_SPEED, PID_SPEED);

    pidController.setPositionPIDWrappingEnabled(false);

    rightMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.follow(leftMotor, true);

    setToZero();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shoulder Encoder", absoluteEncoder.getPosition());
    //SmartDashboard.putNumber("Shoulder zero offset", absoluteEncoder.getZeroOffset());
    //SmartDashboard.putNumber("Shoulder Abs Velocity", absoluteEncoder.getVelocity());
    //SmartDashboard.putNumber("Shoulder Abs applied output", leftMotor.getAppliedOutput());
  }

  public void manualControl(final double direction) {
    final double LIMIT = 0.25;

    if (direction > LIMIT) {
      up();
    } else if (direction < -LIMIT) {
      down();
    } else {
      leftMotor.set(0);
    }
  }

  private void up() {
    leftMotor.set(-SPEED);
  }

  private void down() {
    leftMotor.set(SPEED);
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void setToNinetyDegrees() {
    pidController.setReference(0.37, CANSparkMax.ControlType.kPosition);
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

  /*
  public void adjustReferencePoint() {
    double currentPosition = absoluteEncoder.getPosition();
  }
  */

  public void setToZero() {
    leftMotor.set(0);
  }
}
