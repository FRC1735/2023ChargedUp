// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combos;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CancelAllCommand extends InstantCommand {
  private ScoreHigh scoreHigh;
  private Storage storage;
  private PickupFront pickupFront;
  private ScoreMid scoreMid;
  private HumanPlayerStation humanPlayerStation;

  public CancelAllCommand(ScoreHigh scoreHigh, Storage storage, PickupFront pickupFront, ScoreMid scoreMid, HumanPlayerStation humanPlayerStation) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.scoreHigh = scoreHigh;
    this.storage = storage;
    this.pickupFront = pickupFront;
    this.scoreMid = scoreMid;
    this.humanPlayerStation = humanPlayerStation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    scoreHigh.cancel();
    storage.cancel();
    pickupFront.cancel();
    scoreMid.cancel();
    humanPlayerStation.cancel();
  }
}
