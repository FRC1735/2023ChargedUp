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

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShoulderConstants;

public class Shoulder extends SubsystemBase {
  CANSparkMax leftMotor;
  CANSparkMax rightMotor;

  private final double SPEED = 0.1;
  private final double PID_SPEED = 0.2;
  private SparkMaxAbsoluteEncoder absoluteEncoder;
  private SparkMaxPIDController pidController;


  /** Creates a new Shoulder. */
  public Shoulder() {
    leftMotor = new CANSparkMax(ShoulderConstants.leftCanId, MotorType.kBrushless);
    rightMotor = new CANSparkMax(ShoulderConstants.rightCanId, MotorType.kBrushless);

    leftMotor.setIdleMode(IdleMode.kBrake);
    absoluteEncoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder.setInverted(false);
    //absoluteEncoder.
    absoluteEncoder.setZeroOffset(.6);
    pidController = leftMotor.getPIDController();
    pidController.setFeedbackDevice(absoluteEncoder);

    /*
    pidController.setP(4);
    pidController.setI(0.0004);
    pidController.setD(2);
    pidController.setFF(0);
    pidController.setOutputRange(-PID_SPEED, PID_SPEED);
    */

    pidController.setP(1);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setFF(0);
    pidController.setOutputRange(-PID_SPEED, PID_SPEED);

    //pidController.setPositionPIDWrappingEnabled(true);


    rightMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.follow(leftMotor, true);

    setToZero();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shoulder Encoder", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Shoulder zero offset", absoluteEncoder.getZeroOffset());
    SmartDashboard.putNumber("Shoulder Abs Velocity", absoluteEncoder.getVelocity());
    SmartDashboard.putNumber("Shoulder Abs applied output", leftMotor.getAppliedOutput());
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

  // todo: all of these encoder values are now wrong
  /*
  public void humanPlayerStation() {
    pidController.setReference(0.16, CANSparkMax.ControlType.kPosition);
  }

  public void mid() {
    pidController.setReference(0.22, CANSparkMax.ControlType.kPosition);
  }

  public void top() {
    pidController.setReference(0.15, CANSparkMax.ControlType.kPosition);
  }
  */

  public void storage() {
    pidController.setReference(0.96, CANSparkMax.ControlType.kPosition); 
  }

  public void scoreMid() {
    pidController.setReference(0.28, CANSparkMax.ControlType.kPosition);
  }

  public void pickupFront() {
    pidController.setReference(0.75, CANSparkMax.ControlType.kPosition);
  }

  public void scoreHigh() {
    pidController.setReference(0.24, CANSparkMax.ControlType.kPosition);
  }

  public void pickupAbove() {
    pidController.setReference(0.7, CANSparkMax.ControlType.kPosition);
  }

  public void humanPlayerStation() {
    pidController.setReference(0.25, CANSparkMax.ControlType.kPosition);
  }

  public void setToZero() {
    leftMotor.set(0);
  }
}
