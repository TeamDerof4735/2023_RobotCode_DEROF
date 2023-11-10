package frc.robot.commands.AutoCommands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class tresMtsAdelante extends CommandBase {
  DriveTrain driveTrain;

  public tresMtsAdelante(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;

    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    driveTrain.tresMtsAdelante(3.00);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDriveVolts(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
