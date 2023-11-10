package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;


public class ComandoTrayectoriaPrueba extends SequentialCommandGroup {
  PathPlannerTrajectory trajectory = PathPlanner.loadPath("Trayectoria de Prueba", 0.4, 0.3, false);

  public ComandoTrayectoriaPrueba(DriveTrain driveTrain) {
    InstantCommand resetOdometry = new InstantCommand(()->driveTrain.resetOdometry(trajectory.getInitialPose()));
    addCommands(resetOdometry, 
    driveTrain.createCommandForTrajectory(trajectory));
  }
}
