package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }


  public static final class DriveTrainConstants {
    public static int frontLeft_ID = 1;
    public static int frontRight_ID = 2;
    public static int rearLeft_ID = 3;
    public static int rearRight_ID = 4;

    public static final double kP = 1.5908,
    kI = 0,
    kD = 0.05;

    public static final double gearRatio = 5.1;
    public static final double wheelDiameterMts = 0.1524;

    public static final double TRACK_WIDTH_INCHES = 26.037;
    public static final double TRACK_SCRUB_FACTOR = 0.5;

    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(TRACK_WIDTH_INCHES);
    
    public static final double kS = 0.4202,
    kV = 0.73123,
    kA = 0.1182;

    public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(TRACK_SCRUB_FACTOR);

  }
  
  
  public static final class ShooterConstants {
    public static int leftShoother_ID = 5;
    public static int rightShoother_ID = 6;

    public static final double kP = 0.05,
    kI = 0,
    kD = 0.02,
    kFF = 0.0495;

  }

  public static final class ArmConstants {
    public static int Arm_ID = 7;

    public static int puertoEncoder = 1; 

    public static final double gearRatio = 35.6;

    public static final double kP = 0,
    kI = 0,
    kD = 0,
    kS = 0.13611,
    kG = 0, //hacer fuerza masa aceleracion (fma), gravedad 
    kV = 0.011971,
    kA = 0.00031897;

    public static final double maxVelocity = 0;
    public static final double maxAcceleration = 0;

    
  }
}
