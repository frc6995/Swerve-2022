package frc.robot.util.trajectory;

import java.nio.file.Path;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import frc.robot.util.trajectory.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link PathPlannerTrajectory}
 * with a swerve drive.
 *
 * <p>
 * This command outputs the raw desired Swerve Module States
 * ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from
 * those and used in
 * velocity PIDs.
 *
 * <p>
 * The robot angle controller does not follow the angle given by the trajectory
 * but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
@SuppressWarnings("MemberName")
public class PPSwerveControllerCommand extends CommandBase {
    private final Timer m_timer = new Timer();
    private Supplier<PathPlannerTrajectory> m_trajectorySupplier;
    private PathPlannerTrajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final PPHolonomicDriveController m_controller;
    private final Consumer<ChassisSpeeds> m_outputChassisSpeedsRobotRelative;
    private boolean safeToSample = true;

    /**
     * Constructs a new PPSwerveControllerCommand that when executed will follow the
     * provided
     * trajectory. This command will not return output voltages but rather raw
     * module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path-
     * this is left to the user, since it is not appropriate for paths with
     * nonstationary endstates.
     *
     * @param trajectory         The trajectory to follow.
     * @param pose               A function that supplies the robot pose - use one
     *                           of the odometry classes to
     *                           provide this.
     * @param kinematics         The kinematics for the robot drivetrain.
     * @param xController        The Trajectory Tracker PID controller for the
     *                           robot's x position.
     * @param yController        The Trajectory Tracker PID controller for the
     *                           robot's y position.
     * @param thetaController    The Trajectory Tracker PID controller for angle for
     *                           the robot.
     * @param outputModuleStates The raw output module states from the position
     *                           controllers.
     * @param requirements       The subsystems to require.
     */
    @SuppressWarnings("ParameterName")
    public PPSwerveControllerCommand(
            PathPlannerTrajectory trajectory,
            Supplier<Pose2d> pose,
            PPHolonomicDriveController driveController,
            Consumer<ChassisSpeeds> outputChassisSpeedsFieldRelative,
            Subsystem... requirements) {
                this(()->trajectory, pose, driveController, outputChassisSpeedsFieldRelative, requirements);
        
    }

    public PPSwerveControllerCommand(
        Supplier<PathPlannerTrajectory> trajectory,
        Supplier<Pose2d> pose,
        PPHolonomicDriveController driveController,
        Consumer<ChassisSpeeds> outputChassisSpeedsFieldRelative,
        Subsystem... requirements) {
            m_trajectorySupplier = trajectory;
        m_pose = pose;


        m_controller = driveController;

        m_outputChassisSpeedsRobotRelative = outputChassisSpeedsFieldRelative;

        addRequirements(requirements);
        }


    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_trajectory = m_trajectorySupplier.get();
        if(m_trajectory.getStates().size() == 0) {
            this.safeToSample = false;
        }
        else {
            this.safeToSample = true;
        }
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        PathPlannerState desiredState;
        if (safeToSample) {
            double curTime = m_timer.get();
            desiredState = (PathPlannerState) m_trajectory.sample(curTime);
        }
        else {
            desiredState = new PathPlannerState();
            desiredState.holonomicRotation = m_pose.get().getRotation();
            desiredState.poseMeters = m_pose.get();
            desiredState.holonomicAngularVelocityRadPerSec = 0;
        }
            // By passing in the desired state velocity and, we allow the controller to 
            var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState);
            m_outputChassisSpeedsRobotRelative.accept(targetChassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return !safeToSample || m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}
