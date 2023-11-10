package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  CANSparkMax leftShooter = new CANSparkMax(ShooterConstants.leftShoother_ID, MotorType.kBrushless);
  CANSparkMax rightShooter = new CANSparkMax(ShooterConstants.rightShoother_ID, MotorType.kBrushless);

  RelativeEncoder leftShooterEncoder = leftShooter.getEncoder();
  RelativeEncoder rightShooterEncoder = rightShooter.getEncoder();

  SparkMaxPIDController leftController = leftShooter.getPIDController();
  SparkMaxPIDController rightController = rightShooter.getPIDController();

  public Shooter() {
    leftShooter.restoreFactoryDefaults();
    rightShooter.restoreFactoryDefaults();

    leftShooter.setInverted(false);
    rightShooter.setInverted(true);

    leftShooter.setIdleMode(IdleMode.kBrake);
    rightShooter.setIdleMode(IdleMode.kBrake);

    leftController.setP(ShooterConstants.kP);
    leftController.setI(ShooterConstants.kI);
    leftController.setD(ShooterConstants.kD);
    leftController.setFF(ShooterConstants.kFF);

    rightController.setP(ShooterConstants.kP);
    rightController.setI(ShooterConstants.kI);
    rightController.setD(ShooterConstants.kD);
    rightController.setFF(ShooterConstants.kFF);


  }

  @Override
  public void periodic() {

  }

  public void setMotorsPower(double leftPower, double rightPower) {
    leftShooter.set(leftPower);
    rightShooter.set(rightPower);
  }

  public double currentLeftShooterSpeed() {
    return leftShooterEncoder.getVelocity();
  }

  public double currentRightShooterSpeed() {
    return rightShooterEncoder.getVelocity();
  }

  public void leftSetPoint(double setPoint) {
    leftController.setReference(setPoint, ControlType.kVelocity);
  }

  public void rightSetPoint(double setPoint) {
    rightController.setReference(setPoint, ControlType.kVelocity);
  }

  public void dispararCubo() {
    leftSetPoint(325);
    rightSetPoint(325);
  }
}
