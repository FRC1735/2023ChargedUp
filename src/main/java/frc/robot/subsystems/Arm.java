// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  CANSparkMax motor;

  double SPEED = 1;
  private SparkMaxAbsoluteEncoder absoluteEncoder;


  /** Creates a new Arm. */
  public Arm() {
    this.motor = new CANSparkMax(ArmConstants.canId, MotorType.kBrushless);
    absoluteEncoder = this.motor.getAbsoluteEncoder(Type.kDutyCycle);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Encoder", absoluteEncoder.getPosition());

  }

  public void in() {
    motor.set(SPEED);
  }

  public void out() {
    motor.set(-SPEED);
  }

  public void stop() {
    motor.stopMotor();
  }
}
