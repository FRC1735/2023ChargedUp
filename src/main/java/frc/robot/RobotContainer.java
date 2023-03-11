
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.arm.ArmHumanPlayerStationCommand;
import frc.robot.commands.arm.ArmPickupAboveCommand;
import frc.robot.commands.arm.ArmPickupFrontCommand;
import frc.robot.commands.arm.ArmScoreHighCommand;
import frc.robot.commands.arm.ArmScoreMidCommand;
import frc.robot.commands.arm.ArmStorageCommand;
import frc.robot.commands.shoulder.ShoulderHumanPlayerStationCommand;
import frc.robot.commands.shoulder.ShoulderPickupAboveCommand;
import frc.robot.commands.shoulder.ShoulderPickupFrontCommand;
import frc.robot.commands.shoulder.ShoulderScoreHighCommand;
import frc.robot.commands.shoulder.ShoulderScoreMidCommand;
import frc.robot.commands.shoulder.ShoulderStorageCommand;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  protected final Claw claw = new Claw();

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
    operatorController.a().onTrue(new SequentialCommandGroup(
      new ArmStorageCommand(arm),
      new WaitCommand(2),
      new ShoulderStorageCommand(shoulder),
      new WaitCommand(2),
      new InstantCommand(wrist::storage)
    ));

    // Score Mid
    operatorController.b().onTrue(new SequentialCommandGroup(
      new ShoulderScoreMidCommand(shoulder),
      new WaitCommand(2),
      new ArmScoreMidCommand(arm),
      new WaitCommand(2),
      new InstantCommand(wrist::scoreMid)
    ));

    // Pickup Front
    operatorController.x().onTrue(new SequentialCommandGroup(
      new ShoulderPickupFrontCommand(shoulder),
      new WaitCommand(0),
      new ArmPickupFrontCommand(arm),
      new WaitCommand(0),
      new InstantCommand(wrist::pickupFront)
    ));

    // Score High
    operatorController.y().onTrue(new SequentialCommandGroup(
      new ShoulderScoreHighCommand(shoulder),
      new WaitCommand(2),
      new ArmScoreHighCommand(arm),
      new WaitCommand(2),
      new InstantCommand(wrist::scoreHigh)
    ));

    // Pickup Above
    operatorController.start().onTrue(new SequentialCommandGroup(
      new ShoulderPickupAboveCommand(shoulder),
      new WaitCommand(2),
      new ArmPickupAboveCommand(arm),
      new WaitCommand(2),
      new InstantCommand(wrist::pickupAbove)
    ));

    // Pickup Human Player Station
    operatorController.back().onTrue(new SequentialCommandGroup(
      new ShoulderHumanPlayerStationCommand(shoulder),
      new WaitCommand(2),
      new ArmHumanPlayerStationCommand(arm),
      new WaitCommand(2),
      new InstantCommand(wrist::humanPlayerStation)
    ));

    // Extend Arm
    // TODO: Verify that limit is working on this
    operatorController.povUp().onTrue(new InstantCommand(arm::out, arm)).onFalse(new InstantCommand(arm::stop, arm));

    // Retract Arm
    operatorController.povDown().onTrue(new InstantCommand(arm::in, arm)).onFalse(new InstantCommand(arm::stop, arm));

    // Move shoulder up and down
    // TODO: When this is on it interferes with PID control
    /*
    shoulder.setDefaultCommand(
      new RunCommand(() -> {
        shoulder.manualControl(-operatorController.getLeftY());
    }, shoulder));
    */

    // Open Claw
    operatorController.rightBumper().onTrue(new InstantCommand(claw::open, claw));

    // Cone Claw
    operatorController.rightTrigger().onTrue(new InstantCommand(claw::cone, claw));

    // TODO - cube
    //

    // Manual Open Claw
    //operatorController.rightBumper().onTrue(new InstantCommand(claw::manualOpen, claw)).onFalse(new InstantCommand(claw::stop));

    // Manual Close Claw
    //operatorController.rightTrigger().onTrue(new InstantCommand(claw::manualClose, claw)).onFalse(new InstantCommand(claw::stop));

    // Down Wrist
    operatorController.leftBumper().onTrue(new InstantCommand(wrist::down, wrist)).onFalse(new InstantCommand(wrist::stop, wrist));

    // Up Wrist
    operatorController.leftTrigger().onTrue(new InstantCommand(wrist::up, wrist)).onFalse(new InstantCommand(wrist::stop, wrist));
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
