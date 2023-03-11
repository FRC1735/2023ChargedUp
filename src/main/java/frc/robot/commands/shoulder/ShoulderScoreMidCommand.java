package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;

public class ShoulderScoreMidCommand extends CommandBase {
  Shoulder shoulder;
  int reachedSetpoint = 0;
  int SETPOINT_LIMIT = 5;

  public ShoulderScoreMidCommand(Shoulder shoulder) {
    this.shoulder = shoulder;
  }

  @Override
  public void initialize() {
    shoulder.scoreMid();
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    if (shoulder.isAtScoreMid()) {
      reachedSetpoint += 1;
    } else {
      reachedSetpoint = 0;
    }

    if (reachedSetpoint == SETPOINT_LIMIT) {
      System.out.println("JTA - Shoulder Score Mid Finished");
      return true;
    } else {
      return false;
    }
    //return reachedSetpoint == SETPOINT_LIMIT;
  }
}
