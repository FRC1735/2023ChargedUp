// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

import java.util.List;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem drive = new DriveSubsystem();
  protected final Shoulder shoulder = new Shoulder();
  protected final Arm arm = new Arm();
  protected final Wrist wrist = new Wrist();
  private final Claw claw = new Claw();

  private final CommandXboxController driveController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  
  private final CommandXboxController operatorController = 
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  // Possibly causing issues and we are not using currently
  // See "Onboard I2C Causing System Lockups" at
  // https://docs.wpilib.org/en/stable/docs/yearly-overview/known-issues.html
  //private final ColorSensor m_colorSensor = new ColorSensor();

  private double driveSpeedModifier = 1.0;
  private final double SPEED_MODIFIER = 0.5;
  private final double FULL_SPEED = 1.0;

  public RobotContainer() {
    configureDriveController();
    configureOperatorController();

    // TODO - for both autonomous and teleop - start in storage mode
  }

  private void configureBindings() {
    // preset positions

    // human player station
    /*
    m_controllerA.a().onTrue(new ParallelCommandGroup(
      new InstantCommand(m_wrist::humanPlayerStation),
      new InstantCommand(m_shoulder::humanPlayerStation)
      // todo - arm extension
    ));
    */

    /*
    // mid
    m_controllerA.b().onTrue(
    new ParallelCommandGroup(
      new InstantCommand(m_wrist::mid),
      new InstantCommand(m_shoulder::mid)
      // todo - arm extension
    ));

    // high
    m_controllerA.y().onTrue(new ParallelCommandGroup(
      new InstantCommand(m_wrist::top),
      new InstantCommand(m_shoulder::top)
      // todo - arm extension
    ));
    */

    // manual shoulder control
    //m_controllerA.x().whileTrue(new InstantCommand(m_shoulder::up)).onFalse(new InstantCommand(m_shoulder::stop));
    //m_controllerA.a().whileTrue(new InstantCommand(m_shoulder::down)).onFalse(new InstantCommand(m_shoulder::stop));

    // shoulder
    //m_controllerA.x().whileTrue(new InstantCommand(m_shoulder::up)).onFalse(new InstantCommand(m_shoulder::stop));
    //m_controllerA.a().whileTrue(new InstantCommand(m_shoulder::down)).onFalse(new InstantCommand(m_shoulder::stop));

    //m_controllerA.b().whileTrue(new InstantCommand(m_shoulder::setToNinetyDegrees));//.onFalse(new InstantCommand(m_shoulder));
    //m_controllerA.y().whileTrue(new InstantCommand(m_shoulder::setToZero));

    // testing 90 degrees
    /*
    m_controllerA.b().onTrue(new ParallelCommandGroup(
      //new InstantCommand(m_wrist::mid),
      new InstantCommand(m_shoulder::setToNinetyDegrees)
      // todo - arm extension
    ));
    */

    // arm
    //operatorController.x().whileTrue(new InstantCommand(arm::out)).onFalse(new InstantCommand(arm::stop));
    //operatorController.a().whileTrue(new InstantCommand(arm::in)).onFalse(new InstantCommand(arm::stop));

    // wrist

    //m_controllerA.b().whileTrue(new InstantCommand(m_wrist::open));
    //m_controllerA.y().whileTrue(new InstantCommand(m_wrist::close));

    //m_controllerA.x().whileTrue(new InstantCommand(m_wrist::up)).onFalse(new InstantCommand(m_wrist::stop));
    //m_controllerA.a().whileTrue(new InstantCommand(m_wrist::down)).onFalse(new InstantCommand(m_wrist::stop));

    /*
    // claw
    m_controllerB.x().whileTrue(new InstantCommand(m_claw::close)).onFalse(new InstantCommand(m_claw::stop));
    m_controllerB.a().whileTrue(new InstantCommand(m_claw::open)).onFalse(new InstantCommand(m_claw::stop));
    */
  }

  private void configureDriveController() {
    // Swerve drive with left and right joysticks, also applies speed modifier and deadband
    drive.setDefaultCommand(
      new RunCommand(
          () -> drive.drive(
              -MathUtil.applyDeadband(driveController.getLeftY() * driveSpeedModifier, Constants.OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(driveController.getLeftX() * driveSpeedModifier, Constants.OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(driveController.getRightX() * driveSpeedModifier, Constants.OIConstants.kDriveDeadband),
              true,
              true),
      drive));

      // Zero gyro, use if you want to reset field oriented drive
      driveController.rightBumper().onTrue(new InstantCommand(drive::zeroHeading, drive));

      // Apply speed modifier, that is slow down when left bumper held
      driveController.leftBumper().onTrue(new InstantCommand( () -> driveSpeedModifier = SPEED_MODIFIER)).onFalse(new InstantCommand(() -> driveSpeedModifier = FULL_SPEED));
  }

  private void configureOperatorController() {
    // Storage
    operatorController.a().onTrue(new InstantCommand());

    // Score Mid
    operatorController.b().onTrue(new InstantCommand());

    // Pickup Front
    operatorController.x().onTrue(new InstantCommand());

    // Score High
    operatorController.y().onTrue(new InstantCommand());

    // Pickup High
    // TODO: Unclear to me if "start" is the right button
    // TODO: In the XBox button diagram it is referred to as "menu"
    // TODO: Verify before using
    operatorController.start().onTrue(new PrintCommand("Start pressed"));

    // Pickup Human Player Station
    // TODO: Unclear to me if "back" is the right button
    // TODO: In the XBox button diagram it is referred to as "view"
    // TODO: Verify before using
    operatorController.back().onTrue(new PrintCommand("Back pressed"));

    // Extend Arm
    // TODO: Verify that limit is working on this
    operatorController.povUp().onTrue(new InstantCommand(arm::out, arm));

    // Retract Arm
    operatorController.povDown().onTrue(new InstantCommand(arm::in, arm));

    // Move shoulder up and down
    // TODO: Unsure if this is correct
    shoulder.setDefaultCommand(
      new RunCommand(() -> {
        shoulder.manualControl(operatorController.getLeftY());
    }, arm));

    // Open Claw
    operatorController.rightBumper().onTrue(new InstantCommand(claw::open, claw));

    // Close Claw
    // TODO: Haven't used this before but it looks right rightTrigger 
    // TODO: returns true when it is over 50% pressed
    // TODO: Use 
    //         new InstantCommand(claw::close, claw);
    // TODO: after verifying
    operatorController.rightTrigger().onTrue(new PrintCommand("Right Trigger pressed"));

    // Down Wrist
    operatorController.leftBumper().onTrue(new InstantCommand(wrist::down, wrist));

    // Up Wrist
    // TODO: As with right trigger, verify this does what we want
    // TODO: Use 
    //         new InstantCommand(wrist::up, wrist);
    // TODO: after verifying

    operatorController.leftTrigger().onTrue(new PrintCommand("Left Trigger pressed"));
  }

  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drive.drive(0, 0, 0, false, false));
  }
}
