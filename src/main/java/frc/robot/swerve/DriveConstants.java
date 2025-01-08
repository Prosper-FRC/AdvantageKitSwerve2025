package frc.robot.swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class DriveConstants {

    public static final double trackwidthXMeters = 0.61595;
    public static final double trackwidthYMeters = 0.61595;

    public static final double drivebaseRadius =  Math.hypot(trackwidthXMeters / 2.0, trackwidthYMeters / 2.0);;

    public static final int gyroPort = 10;

    public static final double maxLinearSpped = 4.8;
    public static final double mazLinearAcceleration = 9.6;
    public static final double maxRotationalSpeed = maxLinearSpped * drivebaseRadius;
    public static final double maxRadiansPS = Math.toRadians(1320);

    public static final double azimuthGearRatio = 150.0 / 7.0;
    public static final double driveGearRatio = 6.75 / 1.0;
    public static final double wheelRadiusMeters = 5.08 / 100.0;
    public static final double circumfrenceMeters = 2 * Math.PI * wheelRadiusMeters;

    public static final Translation2d[] ModuleTranslations = new Translation2d[] {
        new Translation2d(trackwidthXMeters / 2.0, trackwidthYMeters / 2.0),
        new Translation2d(trackwidthXMeters / 2.0, -trackwidthYMeters / 2.0),
        new Translation2d(-trackwidthXMeters / 2.0, trackwidthYMeters / 2.0),
        new Translation2d(-trackwidthXMeters / 2.0, -trackwidthYMeters / 2.0)
      };
    
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(ModuleTranslations);

    public static final double kDriftRate = 1;

    public static final SwerveModuleHardwareConfig frontLeft = new SwerveModuleHardwareConfig(
      "FrontLeft", 
      11, 
      21, 
      31, 
      Rotation2d.fromRotations(0.173584));
    
    public static final SwerveModuleHardwareConfig frontRight = new SwerveModuleHardwareConfig(
      "FrontRight", 
      12, 
      22,
      32,
      Rotation2d.fromRotations(-0.180420));
   
    public static final SwerveModuleHardwareConfig backLeft = new SwerveModuleHardwareConfig(
      "BackLeft", 
      13, 
      23,
     33, 
      Rotation2d.fromRotations(0.334229));
    
    public static final SwerveModuleHardwareConfig backRight = new SwerveModuleHardwareConfig(
      "BackRight",
      14, 
      24,
      34, 
      Rotation2d.fromRotations(0.110107));

    public static final boolean invertAzimuths = true;

    public static final SwerveModuleControllerConfig azimuthControllerConfig = new SwerveModuleControllerConfig(15, 0, 0, 0, 0, 0, 0);
    public static final SwerveModuleControllerConfig driveControllerConfig = new SwerveModuleControllerConfig(0, 0, 0, 0.16396, 2.3327, 0, 0);

    public static final ModuleLimits MODULE_LIMITS = new ModuleLimits(maxLinearSpped, mazLinearAcceleration, Math.toRadians(660.0));

    public record SwerveModuleHardwareConfig(String name, int drivePort, int azimuthPort, int cancoderPort, Rotation2d offset) {}

    public record SwerveModuleControllerConfig(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

    public record ModuleLimits(double linearSpeed, double linearAcel, double angle) {}
  
    public record SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates) {}
    
}
