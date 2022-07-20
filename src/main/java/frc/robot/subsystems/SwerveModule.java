package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.sim.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {

    /**
     * Class to represent and handle a swerve module
     * A module's state is measured by a CANCoder for the absolute position, integrated CANEncoder for relative position
     * for both rotation and linear movement
     */

    private SwerveModuleState desiredState = new SwerveModuleState();

    private static final double rotationkP = 0.1;
    private static final double rotationkD = 0.5;

    private static final double drivekP = 0.01;

    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;

    private final DutyCycleEncoder magEncoder;
    private final DutyCycleEncoderSim magEncoderSim;

    //absolute offset for the CANCoder so that the wheels can be aligned when the robot is turned on
    private final Rotation2d offset;

    private final SparkMaxPIDController rotationController;
    private final SparkMaxPIDController driveController;

    public SwerveModule(
        int driveMotorId, 
        int rotationMotorId,
        int magEncoderId,
        double measuredOffsetRadians
    ) {

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationMotorId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();

        magEncoder = new DutyCycleEncoder(magEncoderId);
        magEncoder.setDistancePerRotation(2*Math.PI);
        magEncoder.setDutyCycleRange(1.0/4098.0, 4096.0/4098.0); //min and max pulse width from the mag encoder datasheet
        offset = new Rotation2d(measuredOffsetRadians);
        magEncoder.setPositionOffset(offset.getRadians()/(2*Math.PI));

        magEncoderSim = new DutyCycleEncoderSim(magEncoder);
        if(RobotBase.isSimulation()) {
            magEncoderSim.setAbsolutePosition(Units.degreesToRadians(30));
        }

        driveMotor.setIdleMode(IdleMode.kBrake);
        rotationMotor.setIdleMode(IdleMode.kCoast);

        rotationController = rotationMotor.getPIDController();
        driveController = driveMotor.getPIDController();

        rotationController.setP(rotationkP);
        rotationController.setD(rotationkD);

        driveController.setP(drivekP);

        //set the output of the drive encoder to be in meters for linear measurement
        driveEncoder.setPositionConversionFactor(
            Math.PI * (DriveConstants.wheelDiameterMeters) 
            / DriveConstants.driveWheelGearReduction
        );

        //set the output of the drive encoder to be in meters per second for velocity measurement
        driveEncoder.setVelocityConversionFactor(
            (DriveConstants.wheelDiameterMeters) * Math.PI / 60 / DriveConstants.driveWheelGearReduction
        );

        //set the output of the rotation encoder to be in radians
        rotationEncoder.setPositionConversionFactor(2.0 * Math.PI / DriveConstants.rotationWheelGearReduction);

        REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(rotationMotor, DCMotor.getNEO(1));

    }

    public void resetDistance() {

        driveEncoder.setPosition(0.0);

    }

    public double getDriveDistanceMeters() {

        return driveEncoder.getPosition();

    }
    
    public Rotation2d getMagEncoderAngle() {

        double unsignedAngle = magEncoder.getAbsolutePosition();

        return new Rotation2d(unsignedAngle);

    }

    public Rotation2d getCanEncoderAngle() {
        if(RobotBase.isSimulation()) {
            return desiredState.angle;
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
            return desiredState.speedMetersPerSecond;
        }
        else {
            return driveEncoder.getVelocity();
        }
    }


    /**
     * Initialize the integrated mag encoder
     * The mag encoder will read a (magnet offset) + (module offset from forward)
     * But we subtract the magnet offset in the encoder library.
    */
    public void initRotationOffset() {
        rotationEncoder.setPosition(getMagEncoderAngle().getRadians());
    }

    /**
     * Method to set the desired state of the swerve module
     * Parameter: 
     * SwerveModuleState object that holds a desired linear and rotational setpoint
     * Uses PID and a feedforward to control the output
     */
    public void setDesiredStateClosedLoop(SwerveModuleState desiredState) {

        // Save the desired state for reference (Simulation assumes the modules always are at the desired state)
        
        this.desiredState = SwerveModuleState.optimize(desiredState, getCanEncoderAngle());

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
            DriveConstants.driveFF.calculate(this.desiredState.speedMetersPerSecond)
        );

    }

    public void resetEncoders() {

        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(0);

    }
    
}