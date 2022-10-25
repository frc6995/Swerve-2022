package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.AZMTH_ENC_COUNTS_PER_MODULE_REV;
import static frc.robot.Constants.DriveConstants.AZMTH_REVS_PER_ENC_REV;
import static frc.robot.Constants.DriveConstants.BL;
import static frc.robot.Constants.DriveConstants.BR;
import static frc.robot.Constants.DriveConstants.FL;
import static frc.robot.Constants.DriveConstants.FR;
import static frc.robot.Constants.DriveConstants.NUM_MODULES;
import static frc.robot.Constants.DriveConstants.ROBOT_MASS_kg;
import static frc.robot.Constants.DriveConstants.ROBOT_MOI_KGM2;
import static frc.robot.Constants.DriveConstants.WHEEL_BASE_WIDTH_M;
import static frc.robot.Constants.DriveConstants.WHEEL_ENC_COUNTS_PER_WHEEL_REV;
import static frc.robot.Constants.DriveConstants.WHEEL_RADIUS_M;
import static frc.robot.Constants.DriveConstants.WHEEL_REVS_PER_ENC_REV;
import static frc.robot.Constants.DriveConstants.m_kinematics;
import static frc.robot.Constants.DriveConstants.robotToModuleTL;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.util.sim.SimGyroSensorModel;
import frc.robot.util.sim.wpiClasses.QuadSwerveSim;
import frc.robot.util.sim.wpiClasses.SwerveModuleSim;
import frc.robot.util.trajectory.PPSwerveControllerCommand;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class DrivebaseS extends SubsystemBase implements Loggable {

    /**
     * Subsystem that controls the drivetrain of the robot
     * Handles all the odometry and base movement for the chassis
     */

    /**
     * absolute encoder offsets for the wheels
     * 180 degrees added to offset values to invert one side of the robot so that it doesn't spin in place
     */
    private static final double frontLeftAngleOffset =5.708331;
    private static final double frontRightAngleOffset = 3.783; //11,12
    private static final double rearLeftAngleOffset = 1.787997;
    private static final double rearRightAngleOffset = 5.66;

    /**
     * SwerveModule objects
     * Parameters:
     * drive motor can ID
     * rotation motor can ID
     * external CANCoder can ID
     * measured CANCoder offset
     */

    private final SwerveModule frontLeft = 
        DrivebaseS.swerveModuleFactory(
            CANDevices.frontLeftDriveMotorId,
            CANDevices.frontLeftRotationMotorId,
            CANDevices.frontLeftRotationEncoderId,
            frontLeftAngleOffset,
            "FL"
        );

    private final SwerveModule frontRight = 
        DrivebaseS.swerveModuleFactory(
            CANDevices.frontRightDriveMotorId,
            CANDevices.frontRightRotationMotorId,
            CANDevices.frontRightRotationEncoderId,
            frontRightAngleOffset,
            "FR"
        );

    private final SwerveModule rearLeft = 
        DrivebaseS.swerveModuleFactory(
            CANDevices.rearLeftDriveMotorId,
            CANDevices.rearLeftRotationMotorId,
            CANDevices.rearLeftRotationEncoderId,
            rearLeftAngleOffset,
            "RL"
        );

    private final SwerveModule rearRight = 
        DrivebaseS.swerveModuleFactory(
            CANDevices.rearRightDriveMotorId,
            CANDevices.rearRightRotationMotorId,
            CANDevices.rearRightRotationEncoderId,
            rearRightAngleOffset,
            "RR"
        );

    // commanded values from the joysticks and field relative value to use in AlignWithTargetVision and AlignWithGyro
    private double commandedForward = 0;
    private double commandedStrafe = 0;
    private double commandedRotation = 0;

    private boolean isCommandedFieldRelative = false;

    private final AHRS navx = new AHRS(Port.kMXP);
    private SimGyroSensorModel simNavx = new SimGyroSensorModel();

    public final PIDController xController = new PIDController(3.0, 0, 0);
    public final PIDController yController = new PIDController(3.0, 0, 0);
    public final PIDController thetaController = new PIDController(10, 0, 0);
    public final PPHolonomicDriveController holonomicDriveController = new PPHolonomicDriveController(xController, yController, thetaController);

    /**
     * odometry for the robot, measured in meters for linear motion and radians for rotational motion
     * Takes in kinematics and robot angle for parameters
     */
    private final SwerveDriveOdometry odometry = 
        new SwerveDriveOdometry(
            m_kinematics, 
            new Rotation2d(getHeading().getRadians())
        );

    private final List<SwerveModuleSim> moduleSims = List.of(
        DrivebaseS.swerveSimModuleFactory(),
        DrivebaseS.swerveSimModuleFactory(),
        DrivebaseS.swerveSimModuleFactory(),
        DrivebaseS.swerveSimModuleFactory()
    );

    @Log.Exclude
    private final List<SwerveModule> modules = List.of(
        frontLeft, frontRight,
        rearLeft, rearRight
    );

    private final QuadSwerveSim quadSwerveSim = 
        new QuadSwerveSim(
            WHEEL_BASE_WIDTH_M,
            WHEEL_BASE_WIDTH_M,
            ROBOT_MASS_kg,
            ROBOT_MOI_KGM2,
            moduleSims
            
        );

    public DrivebaseS() {
        navx.reset();
        
        // initialize the rotation offsets for the CANCoders

        // reset the measured distance driven for each module
        frontLeft.resetDistance();
        frontRight.resetDistance();
        rearLeft.resetDistance();
        rearRight.resetDistance();

        // Allow the robot rotation controller to treat crossing over the rollover point as a valid way to move
        // this is useful because if we want to go from 179 to -179 degrees, it's really a 2-degree move, not 358 degrees
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        resetPose(new Pose2d());
    }

    public void driveRotationVolts(int module, double volts) {
        modules.get(module).driveRotationVolts(volts);
    }

    @Override
    public void periodic() {

        // update the odometry every 20ms
        odometry.update(getHeading(), getModuleStates());

    }
    
    public void drive(ChassisSpeeds speeds) {
        // use kinematics (wheel placements) to convert overall robot state to array of individual module states
        SwerveModuleState[] states;

        // If we are stopped (no wheel velocity commanded) then any number of wheel angles could be valid.
        // By default it would point all modules forward when stopped. Here, we override this.
        if(Math.abs(speeds.vxMetersPerSecond) < 0.01
            && Math.abs(speeds.vyMetersPerSecond) < 0.01
            && Math.abs(speeds.omegaRadiansPerSecond) < 0.01) {
                states = getStoppedStates();
        } else {
            // make sure the wheels don't try to spin faster than the maximum speed possible
            states = m_kinematics.toSwerveModuleStates(speeds);
            // NomadMathUtil.normalizeDrive(states, speeds,
            //     Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS,
            //     Constants.DriveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC,
            //     Constants.DriveConstants.MAX_MODULE_SPEED_FPS);
        } 

        setModuleStates(states);

    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, getPoseHeading()));
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
        
        drive(speeds);
        
    }

    public void driveToPose(Pose2d poseRef, double xFF, double yFF, double thetaFF) {
        Pose2d currentPose = getPose();
    // Calculate feedforward velocities (field-relative).

    SmartDashboard.putString("trajPose", poseRef.toString());
    // Calculate feedback velocities (based on position error).
    double xFeedback = xController.calculate(currentPose.getX(), poseRef.getX());
    double yFeedback = yController.calculate(currentPose.getY(), poseRef.getY());
    double thetaFeedback = thetaController.calculate(currentPose.getRotation().getRadians(), poseRef.getRotation().getRadians());

    // Return next output.
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, thetaFeedback, currentPose.getRotation()));
    }

    public void driveToPose(Pose2d poseRef) {
        Pose2d currentPose = getPose();
    // Calculate feedforward velocities (field-relative).

    SmartDashboard.putString("trajPose", poseRef.toString());
    // Calculate feedback velocities (based on position error).
    double xFeedback = xController.calculate(currentPose.getX(), poseRef.getX());
    double yFeedback = yController.calculate(currentPose.getY(), poseRef.getY());
    double thetaFeedback = thetaController.calculate(currentPose.getRotation().getRadians(), poseRef.getRotation().getRadians());
    double xFF = 0;
    double yFF = 0;

    // Return next output.
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, thetaFeedback, currentPose.getRotation()));
    }


    /**
     * Return the desired states of the modules when the robot is stopped. This can be an x-shape to hold against defense,
     * or all modules forward. Here we have it stopping all modules but leaving the angles at their current positions
     * @return
     */
    private SwerveModuleState[] getStoppedStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < NUM_MODULES; i++) {
            states[i] = new SwerveModuleState(0,new Rotation2d(MathUtil.angleModulus(modules.get(i).getCanEncoderAngle().getRadians())));
        }

        return states;
    }

    /**
     * Method to set the desired state for each swerve module
     * Uses PID and feedforward control to control the linear and rotational values for the modules
     */
    public void setModuleStates(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < NUM_MODULES; i++) {
            modules.get(i).setDesiredStateClosedLoop(moduleStates[i]);
        }
    }

    // returns an array of SwerveModuleStates. 
    // Front(left, right), Rear(left, right)
    // This order is important to remain consistent across the codebase, or commands can get swapped around.
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < NUM_MODULES; i++) {
            states[i] = new SwerveModuleState(
                modules.get(i).getCurrentVelocityMetersPerSecond(),
                modules.get(i).getCanEncoderAngle());
        }

        return states;

    }

    /**
     * Return the current position of the robot on field
     * Based on drive encoder and gyro reading
     */
    public Pose2d getPose() {
        if(RobotBase.isSimulation()) {
            return quadSwerveSim.getCurPose();
        }
        else {
            return odometry.getPoseMeters();
        }
    }

    // reset the current pose to a desired pose
    public void resetPose(Pose2d pose) {
        quadSwerveSim.modelReset(pose);
        odometry.resetPosition(pose, getHeading());


    }

    // reset the measured distance driven for each module
    public void resetDriveDistances() {
        modules.forEach((module)->module.resetDistance());
    }

    // return the average distance driven for each module to get an overall distance driven by the robot
    public double getAverageDriveDistanceRadians() {
        double total = 0;
        for (int i = 0; i < NUM_MODULES; i++) {
            total += modules.get(i).getDriveDistanceMeters();
        }
        return total / 4.0;

    }

    // return the average velocity for each module to get an overall velocity for the robot
    public double getAverageDriveVelocityRadiansPerSecond() {
        double total = 0;
        for (int i = 0; i < NUM_MODULES; i++) {
            total += modules.get(i).getCurrentVelocityMetersPerSecond();
        }
        return total / 4.0;
    }

    // get the current heading of the robot based on the gyro
    public Rotation2d getHeading() {
        return navx.getRotation2d();
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
        simNavx.resetToPose(new Pose2d());
    }

    @Override
    public void simulationPeriodic() {
        // set inputs
        if(!DriverStation.isEnabled()){
            for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
                moduleSims.get(idx).setInputVoltages(0.0, 0.0);
            }
        } else {
            for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
                double azmthVolts = modules.get(idx).getAppliedRotationVoltage();
                double wheelVolts = modules.get(idx).getAppliedDriveVoltage();
                moduleSims.get(idx).setInputVoltages(wheelVolts, azmthVolts);
            }
        }

        Pose2d prevRobotPose = quadSwerveSim.getCurPose();

        // Update model (several small steps)
        for (int i = 0; i< 20; i++) {
            quadSwerveSim.update(0.001);
        }
        

        //Set the state of the sim'd hardware
        for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
            double azmthPos = moduleSims.get(idx).getAzimuthEncoderPositionRev();
            azmthPos = azmthPos / AZMTH_ENC_COUNTS_PER_MODULE_REV * 2 * Math.PI;
            double wheelPos = moduleSims.get(idx).getWheelEncoderPositionRev();
            wheelPos = wheelPos / WHEEL_ENC_COUNTS_PER_WHEEL_REV * 2 * Math.PI * WHEEL_RADIUS_M;

            double wheelVel = moduleSims.get(idx).getWheelEncoderVelocityRevPerSec();
            wheelVel = wheelVel / WHEEL_ENC_COUNTS_PER_WHEEL_REV * 2 * Math.PI * WHEEL_RADIUS_M;
            modules.get(idx).setSimState(azmthPos, wheelPos, wheelVel);
            simNavx.update(quadSwerveSim.getCurPose(), prevRobotPose);
        }
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
            getPose().transformBy(new Transform2d(robotToModuleTL.get(FL), getModuleStates()[0].angle))
        );
        field.getObject("frontRight").setPose(
            getPose().transformBy(new Transform2d(robotToModuleTL.get(FR),getModuleStates()[1].angle))
        );
        field.getObject("backLeft").setPose(
            getPose().transformBy(new Transform2d(robotToModuleTL.get(BL), getModuleStates()[2].angle))
        );
        field.getObject("backRight").setPose(
            getPose().transformBy(new Transform2d(robotToModuleTL.get(BR), getModuleStates()[3].angle))
        );
    }

    static SwerveModuleSim swerveSimModuleFactory(){
        return new SwerveModuleSim(DCMotor.getNEO(1), 
                                   DCMotor.getNEO(1), 
                                   WHEEL_RADIUS_M,
                                   1.0/AZMTH_REVS_PER_ENC_REV, // steering motor rotations per wheel steer rotation
                                   1.0/WHEEL_REVS_PER_ENC_REV,
                                   1.0/AZMTH_REVS_PER_ENC_REV, // same as motor rotations because NEO encoder is on motor shaft
                                   1.0/WHEEL_REVS_PER_ENC_REV,
                                   1.3,
                                   0.7,
                                   ROBOT_MASS_kg * 9.81 / QuadSwerveSim.NUM_MODULES, 
                                   0.01 
                                   );
    }

    static SwerveModule swerveModuleFactory(int driveMotorId, int rotationMotorId, int magEncoderId, double measuredOffsetRadians, String name) {
        SwerveModule module = new SwerveModule(driveMotorId, rotationMotorId, magEncoderId, measuredOffsetRadians, name);
        module.resetDistance();
        return module;
    }

    public void resetRelativeRotationEncoders() {
        frontLeft.initRotationOffset();
        frontRight.initRotationOffset();
        rearLeft.initRotationOffset();
        rearRight.initRotationOffset();
    }

    public void resetPID() {
        // xController.reset(odometry.getPoseMeters().getX());
        // yController.reset(odometry.getPoseMeters().getY());
        //thetaController.reset();
    }

    /****COMMANDS */
    public Command pathPlannerCommand(PathPlannerTrajectory path) {
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
            path,
            this::getPose,
            holonomicDriveController,
            this::drive,
            this
        );
        return command;
    }
}