package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmScoreHighCommand extends CommandBase {
  Arm arm;
  int reachedSetpoint = 0;
  int SETPOINT_LIMIT = 5;
  boolean isAuto = false;
  public ArmScoreHighCommand(Arm arm, boolean isAuto) {
    this.arm = arm;
    this.isAuto = isAuto;
  }

  @Override
  public void initialize() {
    if (isAuto) {
      arm.scoreHighAuto();
    }
    else {
      arm.scoreHigh();
    }
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
