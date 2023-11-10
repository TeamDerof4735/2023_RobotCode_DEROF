package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveSignal;
import frc.robot.Constants.DriveTrainConstants;


public class DriveTrain extends SubsystemBase {
  CANSparkMax frontLeft = new CANSparkMax(DriveTrainConstants.frontLeft_ID, MotorType.kBrushless);
  CANSparkMax frontRight = new CANSparkMax(DriveTrainConstants.frontRight_ID, MotorType.kBrushless);
  CANSparkMax rearLeft = new CANSparkMax(DriveTrainConstants.rearLeft_ID, MotorType.kBrushless);
  CANSparkMax rearRight = new CANSparkMax(DriveTrainConstants.rearRight_ID, MotorType.kBrushless);
  
  RelativeEncoder leftEncoder = frontLeft.getEncoder();
  RelativeEncoder rightEncoder = frontRight.getEncoder();

  MotorControllerGroup leftControllers = new MotorControllerGroup(frontLeft, rearLeft);
  MotorControllerGroup rightControllers = new MotorControllerGroup(frontRight, frontRight);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllers, rightControllers);

  AHRS gyro = new AHRS(Port.kMXP);

  PIDController leftPidController = new PIDController(DriveTrainConstants.kP, DriveTrainConstants.kI, DriveTrainConstants.kD);
  PIDController rightPidController = new PIDController(DriveTrainConstants.kP, DriveTrainConstants.kI, DriveTrainConstants.kD);

  DifferentialDriveOdometry odometry = 
  new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()), getLeftMeters(), getRightMeters());

  Field2d field = new Field2d();
  

  public DriveTrain() {

    frontLeft.restoreFactoryDefaults();
    frontRight.restoreFactoryDefaults();
    rearLeft.restoreFactoryDefaults();
    rearRight.restoreFactoryDefaults();

    frontLeft.setInverted(false);
    frontRight.setInverted(true);
    rearLeft.setInverted(false);
    rearRight.setInverted(true);

    frontLeft.setIdleMode(IdleMode.kBrake);
    frontRight.setIdleMode(IdleMode.kBrake);
    rearLeft.setIdleMode(IdleMode.kBrake);
    rearRight.setIdleMode(IdleMode.kBrake);

    
    rearLeft.follow(frontLeft);
    rearRight.follow(frontRight);

    frontLeft.setSmartCurrentLimit(40);
    frontRight.setSmartCurrentLimit(40);
    rearLeft.setSmartCurrentLimit(40);
    rearRight.setSmartCurrentLimit(40);
    
    resetEncoder();
    resetAngle();

    SmartDashboard.putData("Cancha", field);

    // field.setRobotPose(1.75, 4.97, 0.00);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Meters", getLeftMeters());
    SmartDashboard.putNumber("Right Meters", getRightMeters());

    odometry.update(Rotation2d.fromDegrees(getAngle()), getLeftMeters(), getRightMeters());

    field.setRobotPose(
      new Pose2d(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), 
      odometry.getPoseMeters().getRotation()));
    
    SmartDashboard.putNumber("Angulo Actual", getAngle());
  }


  
  public void drive(double distanceController, double turn) {
    differentialDrive.arcadeDrive(distanceController, turn);
  }

  public void setOpenLoop(DriveSignal signal){
    frontLeft.set(signal.getLeft());
    frontRight.set(signal.getRight());
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }
  private static final double kEpsilon = 1E-9;//-------
  public static DriveSignal inverseKinematics(Twist2d velocity) {
    if (Math.abs(velocity.dtheta) < kEpsilon) {
      return new DriveSignal(velocity.dx, velocity.dx);
    }
    double delta_v = DriveTrainConstants.TRACK_WIDTH_INCHES * velocity.dtheta / (2 * DriveTrainConstants.TRACK_SCRUB_FACTOR);
    return new DriveSignal(velocity.dx - delta_v, velocity.dx + delta_v);
  }
  
  public void setCheesyishDrive(Joystick joystick){
    setCheesyishDrive(0.2 * setJoyDeadBand(-joystick.getRawAxis(1), 0.15) ,0.45 * setJoyDeadBand(-joystick.getRawAxis(4), 0.15) , true); //joystick.getRawButton(10));
  }
  
  public static double setJoyDeadBand(double joystickValue, double deadBand) {
    return joystickValue < deadBand && joystickValue > -deadBand ? 0 : joystickValue;
  }

  public void setCheesyishDrive(double throttle, double wheel, boolean quickTurn){
    
    if (epsilonEquals(throttle, 0.0, 0.075)) {
      throttle = 0.0;
    }
    
    if (epsilonEquals(wheel, 0.0, 0.075)) {
      wheel = 0.0;
    }

    final double kWheelGain = 0.05;
    final double kWheelNonlinearity = 0.05;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }
       
    wheel *= kWheelGain;
    DriveSignal signal = inverseKinematics(new Twist2d(throttle, 0.0, wheel));
    double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
    setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
    differentialDrive.feed();
  }


  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftControllers.setVoltage(leftVolts);
    rightControllers.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  public void tankDriveSpeed(double leftSpeed, double rightSpeed) {
    leftControllers.set(leftSpeed);
    rightControllers.set(rightSpeed); 
  }
  
  public double encoderCountsToMts(double encoderCounts) {
    double wheelRotation = encoderCounts / DriveTrainConstants.gearRatio;
    double distance = wheelRotation * (Math.PI * DriveTrainConstants.wheelDiameterMts);
    return distance;
  }

  public void resetEncoder() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public double getLeftMeters() {
    return encoderCountsToMts(leftEncoder.getPosition());
  }

  public double getRightMeters() {
    return encoderCountsToMts(rightEncoder.getPosition());
  }

  public double getAverageMeters(){
    double meters = (getLeftMeters() + getRightMeters()) / 2;
    return meters;
  }

  public double getAngle() {
    return Math.IEEEremainder(gyro.getAngle(), 360) ;
  }

  public void resetAngle() {
    gyro.reset();
  }

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getwWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftMeters(), getRightMeters());
  }

  public void resetOdometry(Pose2d pose2d) {
    resetEncoder();
    resetAngle();

    odometry.resetPosition(Rotation2d.fromDegrees(getAngle()), getLeftMeters(), getRightMeters(), pose2d);
  }

  public Command createCommandForTrajectory(PathPlannerTrajectory trajectory) {
    PPRamseteCommand ramseteCommand = 
    new PPRamseteCommand(
      trajectory, 
      ()-> getPose2d(),
       new RamseteController(2, 0.7), 
        new SimpleMotorFeedforward(DriveTrainConstants.kS, DriveTrainConstants.kV, DriveTrainConstants.kA),
        DriveTrainConstants.driveKinematics, 
         this::getwWheelSpeeds,
          leftPidController, 
           rightPidController, 
            this::tankDriveVolts, 
             this);

    return ramseteCommand;
  }


//PARA AUTONOMO:
  public void treintaCMFrente(double treintaCMFront) {
    
    if(getLeftMeters() < treintaCMFront) {
      tankDriveSpeed(0.5, 0.5);
    } else {
      tankDriveVolts(0, 0);
    }
  }

  public void treintaCMAtras(double treintaCMReverse) {

    if(getLeftMeters() < treintaCMReverse) {
      tankDriveSpeed(-0.5, -0.5);
    } else {
      tankDriveVolts(0, 0);
    }
  }

  public void tresMtsAdelante(double tresMtsAdelante) {

    if(getLeftMeters() < tresMtsAdelante) {
      tankDriveSpeed(0.25, 0.25);
    } else {
      tankDriveVolts(0, 0);
    }
  }

}

