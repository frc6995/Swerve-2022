package frc.robot.util;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class NomadMathUtil {

    public static Rotation2d getDirection(Transform2d transform) {
        return getDirection(transform.getTranslation());
    }

    public static Rotation2d getDirection(Translation2d transform) {
        return new Rotation2d(Math.atan2(transform.getY(), transform.getX()));
    }

    public static double getDistance(Transform2d transform){
        return getDistance(transform.getTranslation());
    }

    public static double getDistance(Translation2d transform) {
        return transform.getNorm();
    }

    public static double calculateDistanceToTargetMeters(
        double cameraHeightMeters,
        double targetHeightMeters,
        double cameraPitchRadians,
        double targetPitchRadians,
        double cameraYawRadians) {
    return (targetHeightMeters - cameraHeightMeters)
            / Math.tan(cameraPitchRadians + targetPitchRadians) / Math.cos(cameraYawRadians);
    }

    /**
     * Gets the rotation of a Rotation2d in the range [0..2pi] instead of the default [-pi..pi]. 
     * 
     * This avoids wrapping problems 
     * @param rotation
     * @return
     */
    public static double modulus(Rotation2d rotation) {
        return modulus(rotation.getRadians());
    }

    public static double modulus(double rotation) {
        if(rotation < 0) {
        rotation = 2*Math.PI + rotation;
        }
        return rotation;
    }

    public static double subtractkS(double voltage, double kS) {
        if(Math.abs(voltage) <= kS) {
            voltage = 0;
        }
        else {
            voltage -= Math.copySign(kS, voltage);
        }
        return voltage;
    }
    /**
     * An improved desaturation algorithm that takes into account the desired chassis speeds
     */
    public static void normalizeDrive(SwerveModuleState[] desiredStates, ChassisSpeeds speeds,
         double kMaxTranslationalVelocity, double kMaxRotationalVelocity, double kModuleMaxSpeedMetersPerSecond) {
        // Determine which of translation or rotation is closer to maximum
        double translationalK = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) / kMaxTranslationalVelocity;
        double rotationalK = Math.abs(speeds.omegaRadiansPerSecond) / kMaxRotationalVelocity;
        double k = Math.max(translationalK, rotationalK);
      
        // Find the how fast the fastest spinning drive motor is spinning                                       
        double realMaxSpeed = 0.0;
        for (SwerveModuleState moduleState : desiredStates) {
          realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }
      

        double scale = Math.min(k * kModuleMaxSpeedMetersPerSecond / realMaxSpeed, 1);
        for (SwerveModuleState moduleState : desiredStates) {
          moduleState.speedMetersPerSecond *= scale;
        }
      }

    
}
