package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends CommandBase {
  
  Shooter shooter;
  Joystick xboxJoystick;

  public IntakeCommand(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.setMotorsPower(-0.4, -0.4);
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



