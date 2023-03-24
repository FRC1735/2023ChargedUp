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
  private PIDController drivePID;
  private double driveDistanceMeters;

  // turning
  private PIDController turnPID;
  private double targetRotation;

  public AutoDrive(DriveSubsystem drive, double driveDistanceFeet, double targetRotation) {
    addRequirements(drive);
    this.driveSubsystem = drive;

    this.drivePID = new PIDController(0.1, 0, 0);
    this.driveDistanceMeters = Units.feetToMeters(driveDistanceFeet);
    
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
    // lol, just noticed we didn't actually use the output in PIDGo and instead always used 0.1
    // TODO - use this x value once we verify we can go slow
    double x = drivePID.calculate(driveSubsystem.m_odometry.getPoseMeters().getX(), driveDistanceMeters);
    
    double xSpeed = 0;
    if (!drivePID.atSetpoint()) {
      xSpeed = 0.1;
    }

    double rot = turnPID.calculate(driveSubsystem.getHeading(), targetRotation);

    driveSubsystem.drive(xSpeed, 0, rot, true, true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return drivePID.atSetpoint() && turnPID.atSetpoint();
  }
}
