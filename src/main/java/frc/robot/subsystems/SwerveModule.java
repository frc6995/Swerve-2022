package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.sim.DutyCycleEncoderSim;
import frc.robot.util.sim.SimEncoder;
import io.github.oblarg.oblog.Loggable;

public class SwerveModule extends SubsystemBase implements Loggable{

    /**
     * Class to represent and handle a swerve module
     * A module's state is measured by a CANCoder for the absolute position, integrated CANEncoder for relative position
     * for both rotation and linear movement
     */

    private SwerveModuleState desiredState = new SwerveModuleState();

    private static final double rotationkP = 0.2;
    private static final double rotationkD = 0.5;

    private static final double drivekP = 0.00;

    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;

    /**
     * Uses meters and m/s
     */
    private final SimEncoder driveEncoderSim;

    /**
     * Uses radians and rad/s
     */
    private final SimEncoder rotationEncoderSim;

    private final DutyCycleEncoder magEncoder;
    private final DutyCycleEncoderSim magEncoderSim;

    //absolute offset for the CANCoder so that the wheels can be aligned when the robot is turned on

    private final SparkMaxPIDController rotationController;
    private final SparkMaxPIDController driveController;
    private final String loggingName;


    public SwerveModule(
        int driveMotorId, 
        int rotationMotorId,
        int magEncoderId,
        double measuredOffsetRadians
    ) {
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationMotorId, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults(false);
        rotationMotor.restoreFactoryDefaults(false);
        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();

        driveEncoderSim = new SimEncoder();
        rotationEncoderSim = new SimEncoder();

        rotationEncoder.setPositionConversionFactor(2.0 * Math.PI * DriveConstants.AZMTH_REVS_PER_ENC_REV);
        //Config the mag encoder, which is directly on the module rotation shaft.

        magEncoder = new DutyCycleEncoder(magEncoderId);
        //magEncoder.setDistancePerRotation(2*Math.PI);
        magEncoder.setDutyCycleRange(1.0/4098.0, 4096.0/4098.0); //min and max pulse width from the mag encoder datasheet

        // The magnet in the module is not aligned straight down the direction the wheel points, but it is fixed in place.
        // This means we can subtract a fixed position offset from the encoder reading,
        // I.E. if the module is at 0 but the magnet points at 30 degrees, we can subtract 30 degrees from all readings
        //magEncoder.setPositionOffset(measuredOffsetRadians/(2*Math.PI));
        
        //Allows us to set what the mag encoder reads in sim.
        magEncoderSim = new DutyCycleEncoderSim(magEncoder);

        //Drive motors should brake, rotation motors should coast (to allow module realignment)
        driveMotor.setIdleMode(IdleMode.kBrake);
        rotationMotor.setIdleMode(IdleMode.kCoast);

        // Config the pid controllers
        rotationController = rotationMotor.getPIDController();
        driveController = driveMotor.getPIDController();

        // For a position controller we use a P loop on the position error
        // and a D loop, which is P on the derivative/rate of change of the position error
        // Theoretically, if the error is increasing (aka, the setpoint is getting away),
        // we should match the velocity of the setpoint with our D term to stabilize the error,
        // then add the additional output proportional to the size of the error.
        rotationController.setP(rotationkP);
        rotationController.setD(rotationkD);

        // For a velocity controller we just use P
        // (and feedforward, which is handled in #setDesiredStateClosedLoop)
        driveController.setP(drivekP);

        //set the output of the drive encoder to be in meters (instead of motor rots) for linear measurement
        // wheel diam * pi = wheel circumference (meters/wheel rot) *
        // 1/6.86 wheel rots per motor rot *
        // number of motor rots
        // = number of meters traveled

        driveEncoder.setPositionConversionFactor(
            Math.PI * (DriveConstants.WHEEL_RADIUS_M * 2) // meters/ wheel rev
            / DriveConstants.WHEEL_ENC_COUNTS_PER_WHEEL_REV // 1/ (enc revs / wheel rev) = wheel rev/enc rev
        );

        //set the output of the drive encoder to be in meters per second (instead of motor rpm) for velocity measurement
        // wheel diam * pi = wheel circumference (meters/wheel rot) *
        // 1/60 minutes per sec *
        // 1/5.14 wheel rots per motor rot *
        // motor rpm = wheel speed, m/s
        driveEncoder.setVelocityConversionFactor(
            (DriveConstants.WHEEL_RADIUS_M * 2) * Math.PI / 60 / DriveConstants.WHEEL_ENC_COUNTS_PER_WHEEL_REV
        );

        //set the output of the rotation encoder to be in radians
        // (2pi rad/(module rotation)) / 12.8 (motor rots/module rots)

        loggingName = "SwerveModule[" + driveMotor.getDeviceId() + ',' + rotationMotor.getDeviceId() + ']';
    }

    public String configureLogName() {
        return "SwerveModule[" + driveMotor.getDeviceId() + ',' + rotationMotor.getDeviceId() + ']';
    }
    /**
     * Reset the driven distance to 0.
     */
    
    public void resetDistance() {

        driveEncoder.setPosition(0.0);
        driveEncoderSim.setPosition(0);

    }

    /**
     * Returns the distance driven by the module in meters since the last reset.
     * @return the distance in meters.
     */
    public double getDriveDistanceMeters() {
        if (RobotBase.isSimulation()) {
            return driveEncoderSim.getPosition();
        }
        else {
            return driveEncoder.getPosition();
        }

    }

    public void driveRotationVolts(double volts) {
        rotationMotor.setVoltage(volts);
    }
    
    /**
     * Returns the current angle of the module in radians, from the mag encoder.
     * @return a Rotation2d, where 0 is forward and pi/-pi is backward.
     */
    public Rotation2d getMagEncoderAngle() {

        double unsignedAngle = magEncoder.getAbsolutePosition() * 2*Math.PI;

        return new Rotation2d(unsignedAngle);

    }

    /**
     * Returns the current angle of the module in radians, from the rotation NEO built-in encoder.
     * The sim model is immediate and perfect response, which is to say that in sim,
     * current angle is always desired angle.
     * @return a Rotation2d, where 0 is forward and pi/-pi is backward.
     */
    public Rotation2d getCanEncoderAngle() {
        if(RobotBase.isSimulation()) {
            return new Rotation2d(rotationEncoderSim.getPosition());
        }
        else {
            return new Rotation2d(rotationEncoder.getPosition());
        }
    }

    /**
     * Returns the current velocity of the module in meters per second.
     * The sim model is immediate and perfect response, which is to say that in sim,
     * current velocity is always desired velocity.
     * @return
     */
    public double getCurrentVelocityMetersPerSecond() {
        if(RobotBase.isSimulation()) {
            return driveEncoderSim.getVelocity();
        }
        else {
            return driveEncoder.getVelocity();
        }
    }

    public double getAppliedDriveVoltage() {
        return driveMotor.getAppliedOutput();
    }

    public double getAppliedRotationVoltage() {
        return rotationMotor.getAppliedOutput();
    }


    /**
     * Initialize the integrated mag encoder
     * The mag encoder will read a (magnet offset) + (module offset from forward)
     * But we subtract the magnet offset in the encoder library, so when starting up, the encoder will report
     * the module offset from forward.
    */
    public void initRotationOffset() {
        rotationEncoder.setPosition(getMagEncoderAngle().getRadians());
        rotationEncoderSim.setPosition(getMagEncoderAngle().getRadians());
    }

    /**
     * Method to set the desired state of the swerve module
     * Parameter: 
     * SwerveModuleState object that holds a desired linear and rotational setpoint
     * Uses PID and a feedforward to control the output
     */
    public void setDesiredStateClosedLoop(SwerveModuleState desiredState) {

        // Save the desired state for reference (Simulation assumes the modules always are at the desired state)
        
        //desiredState = SwerveModuleState.optimize(desiredState, getCanEncoderAngle());
        this.desiredState = desiredState;

        if(RobotBase.isReal()) {
            // Feed the angle to the on-MAX rotation position PID
        rotationController.setReference(
            this.desiredState.angle.getRadians(),
            ControlType.kPosition
        );

        // Feed the speed to the drive MAX velocity PID
        driveController.setReference(
            this.desiredState.speedMetersPerSecond, 
            ControlType.kVelocity,
            0,
            DriveConstants.driveFeedForward.calculate(this.desiredState.speedMetersPerSecond)
        );

        }
        else {
            rotationMotor.setVoltage(rotationkP * 10 * this.desiredState.angle.minus(getCanEncoderAngle()).getRadians());
            driveMotor.setVoltage(DriveConstants.driveFeedForward.calculate(this.desiredState.speedMetersPerSecond));
        }
    }

    public void periodic() {
        SmartDashboard.putNumber("/Shuffleboard/DrivebaseS/"+loggingName+"/magEncoder", getMagEncoderAngle().getRadians());
        SmartDashboard.putNumber("/Shuffleboard/DrivebaseS/"+loggingName+"/rotEncoder", getCanEncoderAngle().getRadians());
        SmartDashboard.putNumber("/Shuffleboard/DrivebaseS/"+loggingName+"/driveEncoder", getDriveDistanceMeters());
        SmartDashboard.putNumber("/Shuffleboard/DrivebaseS/"+loggingName+"/driveVelocity", getCurrentVelocityMetersPerSecond());
        SmartDashboard.putNumber("/Shuffleboard/DrivebaseS/"+loggingName+"/desiredAngle", this.desiredState.angle.getRadians());
        SmartDashboard.putNumber("/Shuffleboard/DrivebaseS/"+loggingName+"/desiredVelocity",this.desiredState.speedMetersPerSecond);
    }

    /**
     * Resets drive and rotation encoders to 0 position. (in sim and irl)
     */
    public void resetEncoders() {

        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(0);
        driveEncoderSim.setPosition(0);
        rotationEncoderSim.setPosition(0);

    }
    
    /**
     * Set the state of the module as specified by the simulator
     * @param angle_rad
     * @param wheelPos_m
     * @param wheelVel_m
     */
    public void setSimState(double angle_rad, double wheelPos_m, double wheelVel_m) {
        rotationEncoderSim.setPosition(angle_rad);
        driveEncoderSim.setPosition(wheelPos_m);
        driveEncoderSim.setPosition(wheelVel_m);
    }
}