// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.Utils;

public class AutoDrive extends CommandBase {
  private DriveSubsystem driveSubsystem;
  
  // driving
  private PIDController xPID;
  private PIDController yPID;
  //private double driveDistanceMeters;

  // turning
  private PIDController turnPID;
  private double targetRotation;

  // uhhhh
  private double xTarget;
  private double yTarget;

  public AutoDrive(DriveSubsystem drive, double xTarget /*double driveDistanceFeet*/, double yTarget, double targetRotation) {
    addRequirements(drive);
    this.driveSubsystem = drive;

    this.xPID = new PIDController(0.1, 0, 0);
    //this.driveDistanceMeters = Units.feetToMeters(driveDistanceFeet);
    this.xTarget = xTarget;

    this.yPID = new PIDController(0.1, 0, 0);
    this.yTarget = yTarget;
    
    this.turnPID = new PIDController(0.001, 0, 0);
    this.targetRotation = targetRotation;
  }

  @Override
  public void initialize() {
    // reset odometry and gyro?
    driveSubsystem.zeroHeading();
    driveSubsystem.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0))); 
  }

  @Override
  public void execute() {

    double xPosition = driveSubsystem.m_odometry.getPoseMeters().getX();
    // lol, just noticed we didn't actually use the output in PIDGo and instead always used 0.1
    // TODO - use this x value once we verify we can go slow
    double x = xPID.calculate(xPosition, xTarget);
    
    double xSpeed = 0;
    if (!xPID.atSetpoint()) {
      xSpeed = -x;//0.1;
    }

    double y = yPID.calculate(driveSubsystem.m_odometry.getPoseMeters().getY(), yTarget);
    double ySpeed = 0;
    if (!yPID.atSetpoint() && xPosition < -0.7) {
      ySpeed = -y;
    }

    double rot = 0;
    //if (driveSubsystem.m_odometry.getPoseMeters().getX() > (driveDistanceMeters / 2) )
    //{
    rot = turnPID.calculate(driveSubsystem.m_gyro.getAngle(), targetRotation);
    //}

    driveSubsystem.drive(xSpeed, ySpeed, -rot, true, true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    boolean xFinished = xPID.atSetpoint();
    boolean yFinished = yPID.atSetpoint();
    boolean turnFinished = turnPID.atSetpoint();

    System.out.println("x: " + xFinished + ", y:" + yFinished + ", turn: " + turnFinished);

    return xFinished && yFinished && turnFinished;
  }
}
