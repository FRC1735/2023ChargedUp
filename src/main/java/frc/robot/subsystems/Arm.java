// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  CANSparkMax motor;

  double SPEED = 1;

  private RelativeEncoder alternateEncoder;


  /** Creates a new Arm. */
  public Arm() {
    this.motor = new CANSparkMax(ArmConstants.canId, MotorType.kBrushless);
    //absoluteEncoder = this.motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle, );

    alternateEncoder = this.motor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
  }

  @Override
  public void periodic() {
    /*
    SmartDashboard.putNumber("Applied Output", motor.getAppliedOutput());
    SmartDashboard.putNumber("Alt Encoder Velocity", alternateEncoder.getVelocity());
    SmartDashboard.putNumber("Alt encoder position", alternateEncoder.getPosition());
    */
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
