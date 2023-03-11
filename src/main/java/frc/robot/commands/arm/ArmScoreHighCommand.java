package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmScoreHighCommand extends CommandBase {
  Arm arm;
  int reachedSetpoint = 0;
  int SETPOINT_LIMIT = 5;

  public ArmScoreHighCommand(Arm arm) {
    this.arm = arm;
  }

  @Override
  public void initialize() {
    arm.scoreHigh();
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    if (arm.isAtScoreHigh()) {
      reachedSetpoint += 1;
    } else {
      reachedSetpoint = 0;
    }

    return reachedSetpoint >= SETPOINT_LIMIT;
  }
}
