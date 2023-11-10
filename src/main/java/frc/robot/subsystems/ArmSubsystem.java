package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends ProfiledPIDSubsystem {
  CANSparkMax armMotor = new CANSparkMax(ArmConstants.Arm_ID, MotorType.kBrushless);

  //DigitalInput input = new DigitalInput(ArmConstants.puertoEncoder);
  DutyCycleEncoder armEncoder = new DutyCycleEncoder(2);
  
  
  ArmFeedforward armFeedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

  public ArmSubsystem() {
    super( 

    new ProfiledPIDController(
            ArmConstants.kP,
            ArmConstants.kI,
            ArmConstants.kD,

            new TrapezoidProfile.Constraints(ArmConstants.maxVelocity, ArmConstants.maxAcceleration)), 90);

    armMotor.restoreFactoryDefaults();
    armMotor.setSmartCurrentLimit(50);
    armMotor.setIdleMode(IdleMode.kBrake);

    armEncoder.setConnectedFrequencyThreshold(1);
    armEncoder.setDutyCycleRange(0, 1);

   
    setGoal(90);

    //armEncoder.setDistancePerRotation(360);
  }

  @Override
  public void periodic() {
    //super.periodic();
    SmartDashboard.putNumber("Angulo Brazo", getMeasurement());
    SmartDashboard.putNumber("Angulo Brazo 2", getAngle());

  }

  public double getAngle()
  {
    return (armEncoder.getAbsolutePosition() * 360);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedForward = armFeedforward.calculate(setpoint.position, setpoint.velocity);

    armMotor.setVoltage(output + feedForward);
  }

  /*@Override
  public double getMeasurement() {
    //Minus 70.5 because that gives us a range betwueen 0-180 degrees, 0 being the left position
    //and 180 the right position while 90 degrees is the idle vertical position
    return armEncoder.getDistance() - 70.5;*/
   

  public double getMeasurement() {
      //Sacar diferencia entre el punto 0 encoder y lo que nosotros queremos que este
      return armEncoder.getAbsolutePosition() * 360;
      }


  public Command goToPosition(double position) {
    Command ejecutable = Commands.runOnce(()-> {
      this.setGoal(position);
      this.enable();
    }, this);
    
    return ejecutable;
  }

  public void manual(double x) {
    armMotor.set(x);
  }
  
}


