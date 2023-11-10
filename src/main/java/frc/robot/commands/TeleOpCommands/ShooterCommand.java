package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LevelSelector;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends CommandBase {

  Shooter shooter;
  LevelSelector levelSelector;

  public ShooterCommand(Shooter shooter, LevelSelector levelSelector) {
    this.shooter = shooter;
    this.levelSelector = levelSelector;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    switch(levelSelector.getLevelName()) {
      case "Low Level": 
        shooter.leftSetPoint(5);
        shooter.rightSetPoint(5); //Cambiar Valores de acuerdo a Nivel Bajo
        break;

      case "Mid Level": 
        shooter.leftSetPoint(50);
        shooter.rightSetPoint(50); //Cambiar Valores de acuerdo a Nivel Medio
        break;

      case "High Level": 
        shooter.leftSetPoint(350);
        shooter.rightSetPoint(350); //Cambiar Valores de acuerdo a Nivel Alto
        break;
      
      case "Presentacion Machine": 
        shooter.leftSetPoint(350);
        shooter.rightSetPoint(350); //Cambiar Valores de acuerdo a Nivel Alto
        break;

      
    }
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


