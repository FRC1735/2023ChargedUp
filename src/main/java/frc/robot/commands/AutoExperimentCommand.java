// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.Utils;

public class AutoExperimentCommand extends CommandBase {
  DriveSubsystem drive;
  boolean isFinished = false;

  /** Creates a new AutoExperimentCommand. */
  public AutoExperimentCommand(DriveSubsystem drive) {
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.zeroHeading();
    drive.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0))); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean shouldRotate = true;
    if (Utils.isCloseEnough(drive.getHeading(), 90) || drive.getHeading() < 90) {
      shouldRotate = false;
      isFinished = true;
    }

    drive.drive(0, 0, shouldRotate ? 0.2 : 0.0, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
