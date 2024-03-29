package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;

public class ShoulderScoreHighCommand extends CommandBase {
  Shoulder shoulder;
  int reachedSetpoint = 0;
  int SETPOINT_LIMIT = 5;

  public ShoulderScoreHighCommand(Shoulder shoulder) {
    this.shoulder = shoulder;
  }

  @Override
  public void initialize() {
    //reachedSetpoint = 0;
    shoulder.scoreHigh();
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    if (shoulder.isAtScoreHigh()) {
      reachedSetpoint += 1;
      //System.out.println("JTA - Shoulder Score High reached setpoint");
    } else {
      reachedSetpoint = 0;
      //System.out.println("JTA - Shoulder Score High DID NOT reached setpoint");
    }

    if (reachedSetpoint >= SETPOINT_LIMIT) {
      //System.out.println("JTA - Shoulder Score High Finished");
      return true;
    } else {
      return false;
    }
    //return reachedSetpoint == SETPOINT_LIMIT;
  }
}
