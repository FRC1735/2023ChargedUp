package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;

public class ShoulderPickupAboveCommand extends CommandBase {
  Shoulder shoulder;
  int reachedSetpoint = 0;
  int SETPOINT_LIMIT = 5;

  public ShoulderPickupAboveCommand(Shoulder shoulder) {
    this.shoulder = shoulder;
  }

  @Override
  public void initialize() {
    shoulder.pickupAbove();
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    if (shoulder.isAtPickupAbove()) {
      reachedSetpoint += 1;
    } else {
      reachedSetpoint = 0;
    }

    return reachedSetpoint >= SETPOINT_LIMIT;
  }
}
