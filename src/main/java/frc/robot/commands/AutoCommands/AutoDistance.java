package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;


public class AutoDistance extends CommandBase {
  DriveTrain driveTrain;
  ArmSubsystem armSubsystem;
  Shooter shooter;  

  private Timer timer = new Timer();

  PIDController drivePidController = new PIDController(DriveTrainConstants.kP, DriveTrainConstants.kI, DriveTrainConstants.kD);
  PIDController armPidController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

  private double distanceToTravel = 5;

  public AutoDistance(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    //this.armSubsystem = armSubsystem;

    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {

    timer.reset();
    timer.start();
    
    drivePidController.setSetpoint(distanceToTravel);
    drivePidController.reset();
    drivePidController.setTolerance(.5);

  }

  
  @Override
  public void execute()  {

  SmartDashboard.putNumber("PID Error", drivePidController.getPositionError());

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
