// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDGo extends PIDCommand {
  /** Creates a new PIDGo. */
  public PIDGo(DriveSubsystem drive, double distance, boolean fieldRelative) {
    super(
        // The controller that the command will use
        new PIDController(0.2, 0, 0),
        // This should return the measurement
        () -> drive.m_odometry.getPoseMeters().getX(),
        // This should return the setpoint (can also be a constant)
        () -> distance,
        // This uses the output
        output -> {
          drive.drive(-output, 0, 0, fieldRelative, true);
        });
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drive);
    // Configure additional PID options by calling `getController` here.

    //drive.zeroHeading();
    //drive.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0))); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
