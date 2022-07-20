package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class DrivebaseS extends SubsystemBase {

    /**
     * Subsystem that controls the drivetrain of the robot
     * Handles all the odometry and base movement for the chassis
     */

    /**
     * absolute encoder offsets for the wheels
     * 180 degrees added to offset values to invert one side of the robot so that it doesn't spin in place
     */
    private static final double frontLeftAngleOffset = Units.degreesToRadians(239.5);
    private static final double frontRightAngleOffset = Units.degreesToRadians(256.7 + 180);
    private static final double rearLeftAngleOffset = Units.degreesToRadians(322.8);
    private static final double rearRightAngleOffset = Units.degreesToRadians(180.0 + 180);

    /**
     * SwerveModule objects
     * Parameters:
     * drive motor can ID
     * rotation motor can ID
     * external CANCoder can ID
     * measured CANCoder offset
     */

    private final SwerveModule frontLeft = 
        new SwerveModule(
            CANDevices.frontLeftDriveMotorId,
            CANDevices.frontLeftRotationMotorId,
            CANDevices.frontLeftRotationEncoderId,
            frontLeftAngleOffset
        );

    private final SwerveModule frontRight = 
        new SwerveModule(
            CANDevices.frontRightDriveMotorId,
            CANDevices.frontRightRotationMotorId,
            CANDevices.frontRightRotationEncoderId,
            frontRightAngleOffset
        );

    private final SwerveModule rearLeft = 
        new SwerveModule(
            CANDevices.rearLeftDriveMotorId,
            CANDevices.rearLeftRotationMotorId,
            CANDevices.rearLeftRotationEncoderId,
            rearLeftAngleOffset
        );

    private final SwerveModule rearRight = 
        new SwerveModule(
            CANDevices.rearRightDriveMotorId,
            CANDevices.rearRightRotationMotorId,
            CANDevices.rearRightRotationEncoderId,
            rearRightAngleOffset
        );

    // commanded values from the joysticks and field relative value to use in AlignWithTargetVision and AlignWithGyro
    private double commandedForward = 0;
    private double commandedStrafe = 0;
    private double commandedRotation = 0;

    private boolean isCommandedFieldRelative = false;

    private final AHRS navx = new AHRS();
    private Rotation2d simGyro = navx.getRotation2d();

    public final ProfiledPIDController rotationController = 
    new ProfiledPIDController(3.0, 0, 0,
        new TrapezoidProfile.Constraints(AutoConstants.maxVelMetersPerSec,
            AutoConstants.maxAccelMetersPerSecondSq
        )
    );

    /**
     * odometry for the robot, measured in meters for linear motion and radians for rotational motion
     * Takes in kinematics and robot angle for parameters
     */
    private final SwerveDriveOdometry odometry = 
        new SwerveDriveOdometry(
            DriveConstants.kinematics, 
            new Rotation2d(getHeading().getRadians())
        );

    public DrivebaseS() {

        navx.reset();

        // initialize the rotation offsets for the CANCoders
        frontLeft.initRotationOffset();
        frontRight.initRotationOffset();
        rearLeft.initRotationOffset();
        rearRight.initRotationOffset();

        // reset the measured distance driven for each module
        frontLeft.resetDistance();
        frontRight.resetDistance();
        rearLeft.resetDistance();
        rearRight.resetDistance();

        // Allow the robot rotation controller to treat crossing over the rollover point as a valid way to move
        // this is useful because if we want to go from 179 to -179 degrees, it's really a 2-degree move, not 358 degrees
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    public void periodic() {

        // update the odometry every 20ms
        odometry.update(getHeading(), getModuleStates());

        SmartDashboard.putNumber("heading", getHeading().getDegrees());
        SmartDashboard.putNumber("Odometry x", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry y", odometry.getPoseMeters().getY());
        
    }
    
    /**
     * method for driving the robot
     * Parameters:
     * forward linear value
     * sideways linear value
     * rotation value
     * if the control is field relative or robot relative
     */
    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

        // update the drive inputs for use in AlignWithGyro and AlignWithTargetVision control
        commandedForward = forward;
        commandedStrafe = strafe;
        commandedRotation = rotation;

        isCommandedFieldRelative = isFieldRelative;
        SmartDashboard.putNumber("desiredRotSpeed", commandedRotation);

        /**
         * ChassisSpeeds object to represent the overall state of the robot
         * ChassisSpeeds takes a forward and sideways linear value and a rotational value
         * 
         * speeds is set to field relative or default (robot relative) based on parameter
         */
        ChassisSpeeds speeds =
            isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    forward, strafe, rotation, getPoseHeading())
                : new ChassisSpeeds(forward, strafe, rotation);
        
        // use kinematics (wheel placements) to convert overall robot state to array of individual module states
        SwerveModuleState[] states;

        // If we are stopped (no wheel velocity commanded) then any number of wheel angles could be valid.
        // By default it would point all modules forward when stopped. Here, we override this.
        if(Math.abs(forward) < 0.05 && Math.abs(strafe) < 0.05 && Math.abs(rotation) < 0.05) {
                states = getStoppedStates();
        } else {
            // make sure the wheels don't try to spin faster than the maximum speed possible
            states = DriveConstants.kinematics.toSwerveModuleStates(speeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxDriveSpeed);
        } 

        
        

        setModuleStates(states);

    }

    /**
     * Return the desired states of the modules when the robot is stopped. This can be an x-shape to hold against defense,
     * or all modules forward. Here we have it stopping all modules but leaving the angles at their current positions
     * @return
     */
    private SwerveModuleState[] getStoppedStates() {
        SwerveModuleState[] states = {
            new SwerveModuleState(0, frontLeft.getCanEncoderAngle()),
            new SwerveModuleState(0, frontRight.getCanEncoderAngle()),
            new SwerveModuleState(0, rearLeft.getCanEncoderAngle()),
            new SwerveModuleState(0, rearRight.getCanEncoderAngle())
        };

        return states;
    }

    /**
     * Method to set the desired state for each swerve module
     * Uses PID and feedforward control to control the linear and rotational values for the modules
     */
    public void setModuleStates(SwerveModuleState[] moduleStates) {

        frontLeft.setDesiredStateClosedLoop(moduleStates[0]);
        frontRight.setDesiredStateClosedLoop(moduleStates[1]);
        rearLeft.setDesiredStateClosedLoop(moduleStates[2]);
        rearRight.setDesiredStateClosedLoop(moduleStates[3]);

    }

    // returns an array of SwerveModuleStates. 
    // Front(left, right), Rear(left, right)
    // This order is important to remain consistent across the codebase, or commands can get swapped around.
    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = {
            new SwerveModuleState(frontLeft.getCurrentVelocityMetersPerSecond(), frontLeft.getCanEncoderAngle()),
            new SwerveModuleState(frontRight.getCurrentVelocityMetersPerSecond(), frontRight.getCanEncoderAngle()),
            new SwerveModuleState(rearLeft.getCurrentVelocityMetersPerSecond(), rearLeft.getCanEncoderAngle()),
            new SwerveModuleState(rearRight.getCurrentVelocityMetersPerSecond(), rearRight.getCanEncoderAngle())
        };

        return states;

    }

    /**
     * Return the current position of the robot on field
     * Based on drive encoder and gyro reading
     */
    public Pose2d getPose() {

        return odometry.getPoseMeters();

    }

    // reset the current pose to a desired pose
    public void resetPose(Pose2d pose) {
        odometry.resetPosition(pose, getHeading());


    }

    // reset the measured distance driven for each module
    public void resetDriveDistances() {

        frontLeft.resetDistance();
        frontRight.resetDistance();
        rearLeft.resetDistance();
        rearRight.resetDistance();

    }

    // return the average distance driven for each module to get an overall distance driven by the robot
    public double getAverageDriveDistanceRadians() {

        return ((
            Math.abs(frontLeft.getDriveDistanceMeters())
            + Math.abs(frontRight.getDriveDistanceMeters())
            + Math.abs(rearLeft.getDriveDistanceMeters())
            + Math.abs(rearRight.getDriveDistanceMeters())) / 4.0);

    }

    // return the average velocity for each module to get an overall velocity for the robot
    public double getAverageDriveVelocityRadiansPerSecond() {

        return ((
            Math.abs(frontLeft.getCurrentVelocityMetersPerSecond())
            + Math.abs(frontRight.getCurrentVelocityMetersPerSecond()) 
            + Math.abs(rearLeft.getCurrentVelocityMetersPerSecond()) 
            + Math.abs(rearRight.getCurrentVelocityMetersPerSecond())) / 4.0);

    }

    // get the current heading of the robot based on the gyro
    public Rotation2d getHeading() {
        if( RobotBase.isSimulation()) {
            return simGyro;
        } else {
            return navx.getRotation2d();
        }
    }

    // Gets the current heading based on odometry. (this value will reflect odometry resets)
    public Rotation2d getPoseHeading() {
        return getPose().getRotation();
    }

    public double[] getCommandedDriveValues() {

        double[] values = {commandedForward, commandedStrafe, commandedRotation};

        return values;

    }

    public boolean getIsFieldRelative() {

        return isCommandedFieldRelative;

    }

    public void resetImu() {
        navx.reset();
        simGyro = new Rotation2d();
    }

    @Override
    public void simulationPeriodic() {
        // Derive the change in gyro heading from the rotation component of the robot speed
        ChassisSpeeds speeds = DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
        double rotationalSpeed = speeds.omegaRadiansPerSecond;
        SmartDashboard.putNumber("rotSpeed", rotationalSpeed);
        SmartDashboard.putNumber("fwdSpeed", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("leftSpeed", speeds.vyMetersPerSecond);
        simGyro = simGyro.plus(new Rotation2d(rotationalSpeed * 0.02)); // Add the rotation traveled in one cycle (0.02 s).
        SmartDashboard.putNumber("simGyro", simGyro.getRadians());
    }

    /**
     * A convenience method to draw the robot pose and 4 poses representing the wheels onto the field2d.
     * @param field
     */
    public void drawRobotOnField(Field2d field) {
        field.setRobotPose(getPose());
        // Draw a pose that is based on the robot pose, but shifted by the translation of the module relative to robot center,
        // then rotated around its own center by the angle of the module.
        field.getObject("frontLeft").setPose(
            getPose().transformBy(new Transform2d(DriveConstants.frontLeftTranslation, getModuleStates()[0].angle))
        );
        field.getObject("frontRight").setPose(
            getPose().transformBy(new Transform2d(DriveConstants.frontRightTranslation,getModuleStates()[1].angle))
        );
        field.getObject("backLeft").setPose(
            getPose().transformBy(new Transform2d(DriveConstants.rearLeftTranslation, getModuleStates()[2].angle))
        );
        field.getObject("backRight").setPose(
            getPose().transformBy(new Transform2d(DriveConstants.rearRightTranslation, getModuleStates()[3].angle))
        );
    }

}