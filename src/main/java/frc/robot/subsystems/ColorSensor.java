// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.CollationKey;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
  private final ColorSensorV3 colorsensor; 
  /** Creates a new ColorSensor. */
  public ColorSensor() { 
    colorsensor = new ColorSensorV3(Port.kOnboard);
  }

  @Override
  public void periodic() {
    /*
    SmartDashboard.putNumber("RED", colorsensor.getRed());
    SmartDashboard.putNumber("GREEN", colorsensor.getGreen());
    SmartDashboard.putNumber("BLUE", colorsensor.getBlue());
    SmartDashboard.putNumber("PROXIMITY", colorsensor.getProximity());
    SmartDashboard.putString("COLOR",colorsensor.getColor().toHexString());

    SmartDashboard.putBoolean("isCube", isCube());
    SmartDashboard.putBoolean("isCone", isCone());
    */

  }

  public boolean isCube() {
    if (colorsensor.getRed() > 300 && colorsensor.getBlue() > 900) {
      return true;
    }
    return false;
  }
  
  public boolean isCone() {
    if (colorsensor.getRed() > 1500 && colorsensor.getRed() < 300
     && colorsensor.getGreen() > 4000 
     && colorsensor.getBlue() > 900 && colorsensor.getBlue() < 900) {
      return true;
    }
    return false;
  }
}
