package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public class SparkDriveConstants {

  public static int kFrontLeftDriveMotorPort = 5;
  public static int kFrontLeftTurningMotorPort = 6;

  public static int kFrontRightDriveMotorPort = 3;
  public static int kFrontRightTurningMotorPort = 4;

  public static int kBackLeftDriveMotorPort = 7;
  public static int kBackLeftTurningMotorPort = 8;

  public static int kBackRightDriveMotorPort = 1;
  public static int kBackRightTurningMotorPort = 2;

  public static int kFrontLeftAbsoluteEncoderPort = 0;
  public static int kFrontRightAbsoluteEncoderPort = 1;
  public static int kBackLeftAbsoluteEncoderPort = 2;
  public static int kBackRightAbsoluteEncoderPort = 3;

  public static Distance kWheelRadius = Inches.of(2.0);

  public static double kDriveGearRatio = 6.746031746031747;
  public static int kDriveCurrentLimit = 80;
  public static double kDriveRotationstoRadians = (2.0 * Math.PI) / kDriveGearRatio;
  public static double kDriveRPMtoRadiansPerSecond = kDriveRotationstoRadians / 60.0;

  public static double kDriveKP = 0.01;
  public static double kDriveKI = 0.0;
  public static double kDriveKD = 0.0;
  public static double kDriveKS = 0.0;
  public static double kDriveKV = 0.0;
  public static double kDriveKA = 0.0;

  public static double kTurningGearRatio = 21.428571428571427;
  public static int kTurningCurrentLimit = 20;
  public static double kTurningRotationstoRadians = (2.0 * Math.PI) / kTurningGearRatio;
  public static double kTurningRPMtoRadiansPerSecond = kTurningRotationstoRadians / 60.0;

  public static double kTurningKP = 5.0;
  public static double kTurningKI = 0.0;
  public static double kTurningKD = 0.5;
  public static double kTurningKS = 0.1;
  public static double kTurningKV = 0.0;
  public static double kTurningKA = 0.0;

  public static boolean kFrontLeftAbsoluteEncoderReversed = false;
  public static boolean kFrontRightAbsoluteEncoderReversed = false;
  public static boolean kBackLeftAbsoluteEncoderReversed = false;
  public static boolean kBackRightAbsoluteEncoderReversed = false;

  public static boolean kFrontLeftTurningEncoderReversed = true;
  public static boolean kFrontRightTurningEncoderReversed = true;
  public static boolean kBackLeftTurningEncoderReversed = true;
  public static boolean kBackRightTurningEncoderReversed = true;

  public static boolean kFrontLeftDriveEncoderReversed = false;
  public static boolean kFrontRightDriveEncoderReversed = true;
  public static boolean kBackLeftDriveEncoderReversed = false;
  public static boolean kBackRightDriveEncoderReversed = true;

  public static double kFrontLeftAbsoluteEncoderOffset = -0.317;
  public static double kFrontRightAbsoluteEncoderOffset = 0.372;
  public static double kBackLeftAbsoluteEncoderOffset = 0.405;
  public static double kBackRightAbsoluteEncoderOffset = -0.449;
}
