package frc.robot.commands.AutoCommands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class DispararCubo extends CommandBase {
  Shooter shooter;

  public DispararCubo(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.dispararCubo();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setMotorsPower(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
