// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmScoreHighCommand;
import frc.robot.commands.shoulder.ShoulderScoreHighCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHigh extends SequentialCommandGroup {
  /** Creates a new ScoreHigh. */
  public ScoreHigh(Shoulder shoulder, Wrist wrist, Arm arm, boolean isAuto) {
    addCommands(
      new ShoulderScoreHighCommand(shoulder),
      new InstantCommand(wrist::scoreHigh),
      new ArmScoreHighCommand(arm, isAuto)
    );
  }
}
