// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmHumanPlayerStationCommand;
import frc.robot.commands.shoulder.ShoulderHumanPlayerStationCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HumanPlayerStation extends SequentialCommandGroup {
  /** Creates a new HumanPlayerStation. */
  public HumanPlayerStation(Shoulder shoulder, Arm arm, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShoulderHumanPlayerStationCommand(shoulder),
      new ArmHumanPlayerStationCommand(arm),
      new InstantCommand(wrist::humanPlayerStation)
    );
  }
}
