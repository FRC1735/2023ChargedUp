// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmScoreMidCommand;
import frc.robot.commands.shoulder.ShoulderScoreMidCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreMid extends SequentialCommandGroup {
  /** Creates a new ScoreMid. */
  public ScoreMid(Shoulder shoulder, Wrist wrist, Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShoulderScoreMidCommand(shoulder),
      new InstantCommand(wrist::scoreMid),
      new ArmScoreMidCommand(arm)
    );
  }
}
