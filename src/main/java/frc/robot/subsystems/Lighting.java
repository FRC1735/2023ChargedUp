// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase {
  private AddressableLED ledLeft;
  private AddressableLEDBuffer bufferLeft;

  //private AddressableLED ledRight;
  //private AddressableLEDBuffer bufferRight;

  private int LED_COUNT = 30; // 60

  /** Creates a new Lighting. */
  public Lighting() {
    ledLeft = new AddressableLED(0);
    //ledRight = new AddressableLED(1);
    
    bufferLeft = new AddressableLEDBuffer(LED_COUNT);
    //bufferRight = new AddressableLEDBuffer(LED_COUNT);
    ledLeft.setLength(bufferLeft.getLength());
    //ledRight.setLength(bufferRight.getLength());

    ledLeft.setData(bufferLeft);
    //ledRight.setData(bufferRight);
    ledLeft.start();
    //ledRight.start();

    setColor(0, 255, 0);
  }

  @Override
  public void periodic() {
    double matchTime = DriverStation.getMatchTime();

    if (!DriverStation.isAutonomous() && (matchTime < 10) && (matchTime > 7.9)) {
      setColor(255, 0, 0);
    }
    // This method will be called once per scheduler run
  }

  public void on() {
    setColor(0, 255, 0);
    //ledLeft.start();
    //ledRight.start();
  }

  public void off() {
    setColor(0, 0, 0);
    ledLeft.stop();
    //ledRight.stop();
  }

  public void green() {
    System.out.println("JTA - GREEN");
    setColor(0, 255, 0);
  }

  public void setColor(int r, int g, int b) {

    for (int i = 0; i < bufferLeft.getLength(); i++) {
      bufferLeft.setRGB(i, r, g, b);
      //bufferRight.setRGB(i, r, g, b);
    }
    ledLeft.setData(bufferLeft);
    //ledRight.setData(bufferRight);
  }
}
