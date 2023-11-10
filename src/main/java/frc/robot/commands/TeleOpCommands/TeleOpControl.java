package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class TeleOpControl extends CommandBase {

  DriveTrain driveTrain; //Declaramos la variable local del DriveTrain
  Joystick xboxController; //Declaramos la variable local del Control

  public TeleOpControl(DriveTrain driveTrain, Joystick xboxController) {
    this.driveTrain = driveTrain;
    this.xboxController = xboxController;

    addRequirements(driveTrain);    
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    driveTrain.setCheesyishDrive(xboxController);

    if(xboxController.getRawAxis(2) > 0.15){
      driveTrain.setCheesyishDrive(0.0, xboxController.getRawAxis(2), true);
    }

    if(xboxController.getRawAxis(3) > 0.15) {
      driveTrain.setCheesyishDrive(0.0, -xboxController.getRawAxis(3), true);
    }
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
