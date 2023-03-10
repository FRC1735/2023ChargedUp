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

public class Claw extends SubsystemBase {
  CANSparkMax motor;
  
  private final double SPEED = 0.25;

  private SparkMaxAbsoluteEncoder absoluteEncoder;
  //private SparkMaxPIDController pidController;

  // TODO: Implement encoder
  // TODO: Implement encoder based limits
  
  /** Creates a new Claw. */
  public Claw() {
    this.motor = new CANSparkMax(Constants.ClawConstants.canId, MotorType.kBrushless);
    this.motor.setIdleMode(IdleMode.kBrake);

    absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Claw Encoder", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Claw zero offset", absoluteEncoder.getZeroOffset());
    SmartDashboard.putNumber("Claw Abs Velocity", absoluteEncoder.getVelocity());
    SmartDashboard.putNumber("Claw Abs applied output", motor.getAppliedOutput());
 
  }

  public void open() {
    motor.set(SPEED);
  }

  public void close() {
    motor.set(-SPEED);
  }

  public void stop() {
    motor.stopMotor();
  }
}
