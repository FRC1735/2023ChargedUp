// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnPID extends PIDCommand {

 double targetAngle;
 private DriveSubsystem drive;

  public TurnPID(DriveSubsystem drive, double targetAngle) {
  /** Creates a new TurnPID. */
    super(
        // The controller that the command will use
        new PIDController(0.0065, 0, 0),
        // This should return the measurement
        () -> drive.getHeading(),
        // This should return the setpoint (can also be a constant)
        () -> targetAngle,
        // This uses the output
        output -> {
          // Use the output here
          //System.out.println(output);
          drive.drive(0, 0, -output, true, true);
        });
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drive);
    // Configure additional PID options by calling `getController` here.

    //drive.zeroHeading();

    this.drive = drive;
    this.targetAngle = targetAngle;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return targetAngle > (drive.getGyroAngle() - 5) && targetAngle < (drive.getGyroAngle() + 5);
  }
}
