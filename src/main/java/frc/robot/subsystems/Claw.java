// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  CANSparkMax motor;
  private final double SPEED = 0.25;

  /** Creates a new Claw. */
  public Claw() {
    this.motor = new CANSparkMax(Constants.ClawConstants.canId, MotorType.kBrushless);
    this.motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
