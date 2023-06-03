
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoExperimentCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.PIDGo;
import frc.robot.commands.TurnPID;
import frc.robot.commands.arm.ArmHumanPlayerStationCommand;
import frc.robot.commands.arm.ArmPickupAboveCommand;
import frc.robot.commands.arm.ArmPickupFrontCommand;
import frc.robot.commands.arm.ArmScoreHighCommand;
import frc.robot.commands.arm.ArmScoreMidCommand;
import frc.robot.commands.arm.ArmStorageCommand;
import frc.robot.commands.combos.CancelAllCommand;
import frc.robot.commands.combos.HumanPlayerStation;
import frc.robot.commands.combos.PickupFront;
import frc.robot.commands.combos.ScoreHigh;
import frc.robot.commands.combos.ScoreMid;
import frc.robot.commands.combos.Storage;
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
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

import java.sql.DriverAction;
import java.util.List;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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
  protected final Lighting lighting = new Lighting();

  public final Storage storage = new Storage(arm, wrist, shoulder);
  public final ScoreHigh scoreHigh = new ScoreHigh(shoulder, wrist, arm, false);
  public final PickupFront pickupFront = new PickupFront(shoulder, arm, wrist);
  public final ScoreMid scoreMid = new ScoreMid(shoulder, wrist, arm);
  public final HumanPlayerStation humanPlayerStation = new HumanPlayerStation(shoulder, arm, wrist);
  public final CancelAllCommand cancelAllCommand = new CancelAllCommand(scoreHigh, storage, pickupFront, scoreMid, humanPlayerStation);

  /* 
  public final Command cancelAllCommand = new InstantCommand(() -> {
    scoreHigh.cancel();
    storage.cancel();
    pickupFront.cancel();
    scoreMid.cancel();
    humanPlayerStation.cancel();
  });
  */

  private final CommandXboxController driveController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  
  private final CommandXboxController operatorController = 
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  SendableChooser<Command> autoChooser = new SendableChooser<>();

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

    //new InstantCommand(claw::cone, claw);

    SmartDashboard.putNumber("turn p", 0);
    SmartDashboard.putNumber("turn d", 0);

    autoChooser.setDefaultOption("Do Nothing", autonomousDoNothingCommand);
    autoChooser.addOption("Score High, Back Up", autonomousDropConeAtHighThenMoveBack);
    autoChooser.addOption("Score High", autonomousDropConeAtHigh);
    autoChooser.addOption("Back Up", autonomousGoBackCommand);
    autoChooser.addOption("New Auto Right", autoNewWIPCommand);
    autoChooser.addOption("New Auto Left", autoNewWIPCommandLeft);
    SmartDashboard.putData(autoChooser);

    lighting.on();
    lighting.setColor(255,0,0);

    //SmartDashboard.putData(scoreHigh);

    //SmartDashboard.putData(CommandScheduler.getInstance());
    
    //SmartDashboard.putData(shoulder);
    //SmartDashboard.putData(claw);
    //SmartDashboard.putData(wrist);
    //SmartDashboard.putData(arm);

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
  

      // LEDs
      driveController.y().onTrue(new InstantCommand(lighting::blank, lighting));
      driveController.x().onTrue(new InstantCommand(lighting::green, lighting));
      driveController.a().onTrue(new InstantCommand(lighting::yellow, lighting));
      driveController.b().onTrue(new InstantCommand(lighting::purple, lighting));
          
      
      // Storage


      driveController.a().onTrue(new InstantCommand(
        () -> {
          drive.resetDisplacement();
        }
      ));

    /*driveController.a().onTrue(new SequentialCommandGroup(
      new InstantCommand(claw::cone, claw),
      new WaitCommand(1),
      new ArmStorageCommand(arm),
      new WaitCommand(1),
      new InstantCommand(wrist::storage),
      new WaitCommand(1),
      new ShoulderStorageCommand(shoulder)
    ));*/
  
  }

  private void configureOperatorController() {
    // Storage
    operatorController.a().onTrue(storage);

    // Score Mid
    operatorController.b().onTrue(scoreMid);

    // Pickup Front
    operatorController.x().onTrue(pickupFront);

    // Score High
    operatorController.y().onTrue(scoreHigh);

    // Pickup Above
    /* 
    operatorController.start().onTrue(new SequentialCommandGroup(
      new ShoulderPickupAboveCommand(shoulder),
      new WaitCommand(2),
      new ArmPickupAboveCommand(arm),
      new WaitCommand(2),
      new InstantCommand(wrist::pickupAbove)
    ));
    */

    // Pickup Human Player Station
    operatorController.back().onTrue(humanPlayerStation);

    // Extend Arm
    // TODO: Verify that limit is working on this
    operatorController.povUp().onTrue(new InstantCommand(arm::out, arm)).onFalse(new InstantCommand(arm::stop, arm));

    // Retract Arm
    operatorController.povDown().onTrue(new InstantCommand(arm::in, arm)).onFalse(new InstantCommand(arm::stop, arm));

    // Cancel score high command
    operatorController.povRight().onTrue(new InstantCommand(() -> {
      scoreHigh.cancel();
      storage.cancel();
      pickupFront.cancel();
      scoreMid.cancel();
      humanPlayerStation.cancel();
    }));

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
    //operatorController.rightBumper().onTrue(new InstantCommand(claw::manualOpen, claw)).onFalse(new InstantCommand(claw::stop, claw));


    // Cone Claw
    operatorController.rightTrigger().onTrue(new InstantCommand(claw::cone, claw));
    //operatorController.rightTrigger().onTrue(new InstantCommand(claw::manualClose, claw)).onFalse(new InstantCommand(claw::stop, claw));



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

  Command autonomousDropConeAtHigh = new SequentialCommandGroup(
    // close claw
    new InstantCommand(claw::cone, claw),
    // storage mode
    new SequentialCommandGroup(
      new ArmStorageCommand(arm),
      new InstantCommand(arm::stop, arm),
      new WaitCommand(0),
      new InstantCommand(wrist::storage),
      new WaitCommand(0),
      new ShoulderStorageCommand(shoulder)
    ),
    // score high
    new SequentialCommandGroup(
      new ShoulderScoreHighCommand(shoulder),
      new WaitCommand(0),
      new InstantCommand(wrist::scoreHigh),
      new WaitCommand(0),
      new ArmScoreHighCommand(arm, true)
    ),
    // release cone
    new InstantCommand(claw::open, claw),
    new WaitCommand(1),
    // storage
    new SequentialCommandGroup(
      new ArmStorageCommand(arm),
      new InstantCommand(arm::stop, arm),
      new WaitCommand(0),
      new InstantCommand(wrist::storage),
      new WaitCommand(0),
      new ShoulderStorageCommand(shoulder)
    )
  );

  public Command autoScoreHighRedux = new SequentialCommandGroup(
    // close claw
    new InstantCommand(claw::cone, claw),
    // storage mode
    /*
    new SequentialCommandGroup(
      new ArmStorageCommand(arm),
      new InstantCommand(arm::stop, arm),
      new WaitCommand(0),
      new InstantCommand(wrist::storage),
      new WaitCommand(0),
      new ShoulderStorageCommand(shoulder)
    ),
    */
    // score high
    new SequentialCommandGroup(
      new ShoulderScoreHighCommand(shoulder),
      new InstantCommand(wrist::scoreHigh),
      new ArmScoreHighCommand(arm, true)
    ),
    // release cone
    new InstantCommand(claw::open, claw),
    new WaitCommand(.25)
  );

  public Command autonomousDropConeAtHighThenMoveBack = new SequentialCommandGroup(
      // close claw
      new InstantCommand(claw::cone, claw),
      // storage mode
      new SequentialCommandGroup(
        new ArmStorageCommand(arm),
        new InstantCommand(arm::stop, arm),
        new WaitCommand(0),
        new InstantCommand(wrist::storage),
        new WaitCommand(0),
        new ShoulderStorageCommand(shoulder)
      ),
      // score high
      new SequentialCommandGroup(
        new ShoulderScoreHighCommand(shoulder),
        new WaitCommand(0),
        new InstantCommand(wrist::scoreHigh),
        new WaitCommand(0),
        new ArmScoreHighCommand(arm, true)
      ),
      // release cone
      new InstantCommand(claw::open, claw),
      new WaitCommand(1),
      // storage
      new SequentialCommandGroup(
        new ArmStorageCommand(arm),
        new InstantCommand(arm::stop, arm),
        new WaitCommand(0),
        new InstantCommand(wrist::storage),
        new WaitCommand(0),
        new ShoulderStorageCommand(shoulder)
      ),
      // drive back
      new InstantCommand(drive::zeroHeading, drive),
      new RunCommand(() -> drive.drive(0.5, 0, 0, true, true), drive).withTimeout(1.8)
    );


  // TODO - BAD!
 Command autonomousGoBackCommand = new SequentialCommandGroup(
      new InstantCommand(drive::zeroHeading, drive),
      new RunCommand(() -> drive.drive(0.5, 0, 0, true, true), drive).withTimeout(1.8)
  );

  Command autonomousDoNothingCommand = new WaitCommand(1);

  Command autoNewWIPCommand = new SequentialCommandGroup(autoScoreHighRedux,
  new InstantCommand(drive::zeroHeading, drive),
  new InstantCommand(drive::zeroOdometry, drive),
  new InstantCommand(claw::openForAuto, claw),
  new ParallelCommandGroup(
    // move to cone while going into storaget mode

    // storage mode
    new SequentialCommandGroup(
      new ArmStorageCommand(arm),
      new InstantCommand(arm::stop, arm),
      //new WaitCommand(1),

      // pickup front
        new ShoulderPickupFrontCommand(shoulder),
        new WaitCommand(0),
        new ArmPickupFrontCommand(arm),
        new WaitCommand(0),
        new InstantCommand(wrist::pickupFront)
      
    ),
  
    // move then turn
    new SequentialCommandGroup( 
      new PIDGo(drive, -5.2, true), 
      new ParallelCommandGroup(
        new TurnPID(drive, 15)
      ),
      new PIDGo(drive, -5.2 - 0.6 /*0.6604*/, false),
      new RunCommand(claw::cone, claw).withTimeout(1)
    )
  ),
  new ParallelCommandGroup(
    new PIDGo(drive, -5.2 + 0.75 /*0.6604*/, false),
    // storage mode
    new SequentialCommandGroup(
      new ArmStorageCommand(arm),
      new InstantCommand(arm::stop, arm),
      new WaitCommand(0),
      new InstantCommand(wrist::storage),
      new WaitCommand(0),
      new ShoulderStorageCommand(shoulder)
    )
  )
);

Command autoNewWIPCommandLeft = new SequentialCommandGroup( new SequentialCommandGroup(
  // close claw
  new InstantCommand(claw::cone, claw),
  // storage mode
  /*
  new SequentialCommandGroup(
    new ArmStorageCommand(arm),
    new InstantCommand(arm::stop, arm),
    new WaitCommand(0),
    new InstantCommand(wrist::storage),
    new WaitCommand(0),
    new ShoulderStorageCommand(shoulder)
  ),
  */
  // score high
  new SequentialCommandGroup(
    new ShoulderScoreHighCommand(shoulder),
    new InstantCommand(wrist::scoreHigh),
    new ArmScoreHighCommand(arm, true)
  )),
  // release cone
  new InstantCommand(claw::open, claw),
  new WaitCommand(.25),
new InstantCommand(drive::zeroHeading, drive),
new InstantCommand(drive::zeroOdometry, drive),
new InstantCommand(claw::openForAuto, claw),
new ParallelCommandGroup(
  // move to cone while going into storaget mode

  // storage mode
  new SequentialCommandGroup(
    new ArmStorageCommand(arm),
    new InstantCommand(arm::stop, arm),
    //new WaitCommand(1),

    // pickup front
      new ShoulderPickupFrontCommand(shoulder),
      new WaitCommand(0),
      new ArmPickupFrontCommand(arm),
      new WaitCommand(0),
      new InstantCommand(wrist::pickupFront)
    
  ),

  // move then turn
  new SequentialCommandGroup( 
    new PIDGo(drive, -5.2, true), 
    new ParallelCommandGroup(
      new TurnPID(drive, 345)
    ),
    new PIDGo(drive, -5.2 - 0.6 /*0.6604*/, false),
    new RunCommand(claw::cone, claw).withTimeout(1)
  )
),
new ParallelCommandGroup(
  new PIDGo(drive, -5.2 + 0.75 /*0.6604*/, false),
  // storage mode
  new SequentialCommandGroup(
    new ArmStorageCommand(arm),
    new InstantCommand(arm::stop, arm),
    new WaitCommand(0),
    new InstantCommand(wrist::storage),
    new WaitCommand(0),
    new ShoulderStorageCommand(shoulder)
  )
));

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();


    //return new AutoDrive(drive, -4.5, 11);

    //return new TurnPID(drive);


    //return 
    
                              

    // new TurnPID(drive);
    //new AutoExperimentCommand(drive);
    /* 
    InstantCommand( () -> {   
      drive.zeroHeading();
      drive.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0))); 
    }
  , drive    ).andThen(
    
    new RunCommand(() -> drive.drive(0.2, 0, 0.2, true, true), drive).withTimeout(1.8)
  );
*/




    /*
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
        List.of(new Translation2d(0.25, 0), new Translation2d(0.5, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0.6, 0, new Rotation2d(0)),
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
    return
    new InstantCommand(drive::zeroHeading, drive)
    .andThen(new InstantCommand(drive::flipGyro, drive))
    .andThen(swerveControllerCommand)
    .andThen(() -> drive.drive(0, 0, 0, false, false))
    .andThen();
    */
  }
}
