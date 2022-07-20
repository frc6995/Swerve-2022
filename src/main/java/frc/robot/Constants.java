package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class CANDevices {

        public static final int frontLeftRotationMotorId = 7;
        public static final int frontLeftDriveMotorId = 8;

        public static final int frontRightRotationMotorId = 1;
        public static final int frontRightDriveMotorId = 2;

        public static final int rearLeftRotationMotorId = 6;
        public static final int rearLeftDriveMotorId = 5;

        public static final int rearRightRotationMotorId = 3;
        public static final int rearRightDriveMotorId = 4;


        public static final int frontLeftRotationEncoderId = 17;
        public static final int frontRightRotationEncoderId = 13;
        public static final int rearLeftRotationEncoderId = 15;
        public static final int rearRightRotationEncoderId = 14;

    }

    public static final class InputDevices {

        public static final int gamepadPort = 0;

    }

    public static final class DriveConstants {

        public static final double trackWidth = Units.inchesToMeters(16.5);
        public static final double wheelBase = Units.inchesToMeters(16.5);
        public static final Translation2d frontLeftTranslation  = new Translation2d(trackWidth / 2.0, wheelBase / 2.0); //front left
        public static final Translation2d frontRightTranslation = new Translation2d(trackWidth / 2.0, -wheelBase / 2.0);//front right
        public static final Translation2d rearLeftTranslation   = new Translation2d(-trackWidth / 2.0, wheelBase / 2.0);//rear left
        public static final Translation2d rearRightTranslation  = new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0);//rear right

        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                frontLeftTranslation, 
                frontRightTranslation,
                rearLeftTranslation,
                rearRightTranslation
            );
        
        public static final double driveWheelGearReduction = 6.86;
        public static final double rotationWheelGearReduction = 12.8;

        public static final double wheelDiameterMeters = 0.050686 * 2;

        public static final double rotationMotorMaxSpeedRadPerSec = 1.0;
        public static final double rotationMotorMaxAccelRadPerSecSq = 1.0;

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.254, 0.137);

        public static final double maxDriveSpeed = 14.4;
        public static final double teleopTurnRateDegPerSec = 360.0; //Rate the robot will spin with full rotation command

    }

    
    public static final class VisionConstants {

        public static final double limelightHeightInches = 26.5; // distance from limelight to ground
        public static final double limelightMountAngleRadians = Units.degreesToRadians(44);

    }

    public static final class AutoConstants {

        public static final double maxVelMetersPerSec = 2;
        public static final double maxAccelMetersPerSecondSq = 1;
        
    }

    public static final class FieldConstants {

        public static final double targetHeightInches = 89.5;

    }
    
}
