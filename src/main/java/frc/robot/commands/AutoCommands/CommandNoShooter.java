package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoCommands.AutonomousCommands.treintacmfrente;
import frc.robot.commands.AutoCommands.AutonomousCommands.treintacmreversa;
import frc.robot.commands.AutoCommands.AutonomousCommands.tresMtsAdelante;
import frc.robot.subsystems.DriveTrain;


public class CommandNoShooter extends SequentialCommandGroup {
  DriveTrain driveTrain;
  
  public CommandNoShooter(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;

    addRequirements(driveTrain);

    addCommands(
      new ParallelDeadlineGroup(new WaitCommand(.5),new SequentialCommandGroup(new treintacmfrente(driveTrain))),
      new ParallelDeadlineGroup(new WaitCommand(.5), new SequentialCommandGroup(new treintacmreversa(driveTrain))),
      new ParallelDeadlineGroup(new WaitCommand(5), new SequentialCommandGroup(new tresMtsAdelante(driveTrain)))
    );
  }
}
