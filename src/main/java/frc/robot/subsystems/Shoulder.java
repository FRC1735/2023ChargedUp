// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;

public class Shoulder extends SubsystemBase {
  CANSparkMax leftMotor;
  CANSparkMax rightMotor;

  private final double SPEED = 0.5;

  /** Creates a new Shoulder. */
  public Shoulder() {
    leftMotor = new CANSparkMax(ShoulderConstants.leftCanId, MotorType.kBrushless);
    rightMotor = new CANSparkMax(ShoulderConstants.rightCanId, MotorType.kBrushless);

    // TODO - verify
    leftMotor.setIdleMode(IdleMode.kBrake);
    // TODO - make break mode after testing just left side
    rightMotor.setIdleMode(IdleMode.kCoast);

    // TODO - verify
    rightMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void up() {
    leftMotor.set(SPEED);

    // TODO - enable
    //rightMotor.set(SPEED);
  }

  public void down() {
    leftMotor.set(-SPEED);
       
    // TODO - enable
    //rightMotor.set(-SPEED);
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
