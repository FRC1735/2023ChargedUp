// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  private CANSparkMax motor;
  private final double SPEED = 0.5;

  /** Creates a new Wrist. */
  public Wrist() {
    motor = new CANSparkMax(Constants.WristConstants.canId, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void up() {
    motor.set(SPEED);
  }

  public void down() {
    motor.set(-SPEED);
  }

  public void stop() {
    motor.stopMotor();
  }
}
