package frc.robot;

import frc.robot.commands.AutoCommands.AutoDistance;
//import frc.robot.commands.AutoCommands.AutoDistance;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoCommands.ComandoTrayectoriaPrueba;
import frc.robot.commands.AutoCommands.CommandNoShooter;
import frc.robot.commands.AutoCommands.CommandTiger;
import frc.robot.commands.TeleOpCommands.IntakeCommand;
import frc.robot.commands.TeleOpCommands.ShooterCommand;
import frc.robot.commands.TeleOpCommands.TeleOpControl;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LevelSelector;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SideSelector;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;


public class RobotContainer {
  private Joystick chassisDriver = new Joystick(0);
  private Joystick subsystemsDriver = new Joystick(1);


  private DriveTrain driveTrain = new DriveTrain();
  private Shooter shooter = new Shooter();
  private LevelSelector levelSelector = new LevelSelector(subsystemsDriver);
  private ArmSubsystem arm = new ArmSubsystem();
  private SideSelector sideSelector = new SideSelector(subsystemsDriver);

  private CommandTiger commandTiger = new CommandTiger(driveTrain, shooter);
  private CommandNoShooter commandNoShooter = new CommandNoShooter(driveTrain);


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    //Control 1:
    //new JoystickButton(xboxJoystick, 1).onTrue(new AutoDistance(driveTrain));
    driveTrain.setDefaultCommand(new TeleOpControl(driveTrain, chassisDriver));
    
    //Control 2:
    
    new JoystickButton(subsystemsDriver, 3).whileTrue(new IntakeCommand(shooter));
    
    new JoystickButton(subsystemsDriver, 2).whileTrue(new ShooterCommand(shooter, levelSelector)); 
    
    new POVButton(subsystemsDriver, 0).onTrue(new InstantCommand(()-> levelSelector.selectionUp()));
    new POVButton(subsystemsDriver, 180).onTrue(new InstantCommand(()-> levelSelector.selectionDown()));
    
    arm.setDefaultCommand(new RunCommand(() -> arm.manual(subsystemsDriver.getRawAxis(1)), arm));
    
    new POVButton(subsystemsDriver, 90).onTrue(new InstantCommand(()-> sideSelector.selectionRight()));
    new POVButton(subsystemsDriver, 270).onTrue(new InstantCommand(()-> sideSelector.selectionLeft()));
    

  }

  public Command getAutonomousCommand() {
    return commandTiger;
    //return commandNoShooter;
  }
}
