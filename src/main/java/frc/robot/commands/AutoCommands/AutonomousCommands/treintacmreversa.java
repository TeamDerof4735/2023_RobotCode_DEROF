// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class treintacmreversa extends CommandBase {
  DriveTrain driveTrain;
  
  public treintacmreversa(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;

    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    driveTrain.treintaCMAtras(.30);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
