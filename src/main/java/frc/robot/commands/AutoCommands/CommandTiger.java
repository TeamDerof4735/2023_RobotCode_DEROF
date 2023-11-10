package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoCommands.AutonomousCommands.DispararCubo;
import frc.robot.commands.AutoCommands.AutonomousCommands.tresMtsAdelante;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class CommandTiger extends SequentialCommandGroup {
  DriveTrain driveTrain;
  Shooter shooter;
  
  public CommandTiger(DriveTrain driveTrain, Shooter shooter) {
    this.driveTrain = driveTrain;
    this.shooter = shooter;

    addRequirements(shooter);
    addRequirements(driveTrain);

    addCommands(
      new ParallelDeadlineGroup(new WaitCommand(.5), new SequentialCommandGroup(new DispararCubo(shooter))),
      new ParallelDeadlineGroup(new WaitCommand(5), new SequentialCommandGroup(new tresMtsAdelante(driveTrain)))

    );
  }
}
