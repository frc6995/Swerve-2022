// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.trajectory;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory} with a swerve drive.
 *
 * <p>This command outputs the raw desired Swerve Module States ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from those and used in
 * velocity PIDs.
 *
 * <p>The robot angle controller does not follow the angle given by the trajectory but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
@SuppressWarnings("MemberName")
public class PoseListSwerveControllerCommand extends CommandBase {
  private final Timer m_timer = new Timer();
  private final List<TrajectoryReader.State> m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final PIDController m_xController;
  private final PIDController m_yController;
  private final ProfiledPIDController m_thetaController;
  private final Consumer<ChassisSpeeds> m_outputSpeeds;

  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param desiredRotation The angle that the drivetrain should be facing. This is sampled at each
   *     time step.
   * @param outputSpeeds The raw output module states from the position controllers.
   * @param requirements The subsystems to require.
   */
  @SuppressWarnings("ParameterName")
  public PoseListSwerveControllerCommand(
      List<TrajectoryReader.State> trajectory,
      Supplier<Pose2d> pose,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Consumer<ChassisSpeeds> outputSpeeds,
      Subsystem... requirements) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
    m_pose = requireNonNullParam(pose, "pose", "SwerveControllerCommand");

    m_xController = requireNonNullParam(xController, "xController", "SwerveControllerCommand");
    m_yController = requireNonNullParam(yController, "yController", "SwerveControllerCommand");
    m_thetaController = requireNonNullParam(thetaController, "thetaController", "SwerveControllerCommand");

    m_outputSpeeds =
        requireNonNullParam(outputSpeeds, "outputSpeeds", "SwerveControllerCommand");

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void execute() {
    int curTime = (int) (m_timer.get() * 50);
    SmartDashboard.putNumber("curTime", curTime);
    TrajectoryReader.State desiredState;
    if(curTime >= m_trajectory.size()) {
      desiredState = new TrajectoryReader.State(curTime,
        m_trajectory.get(m_trajectory.size()-1).poseMeters, 0, 0, 0);
    } else if (curTime < 0) {
      desiredState = new TrajectoryReader.State(curTime,
        m_trajectory.get(0).poseMeters, 0, 0, 0);
    }
    else {
      desiredState = m_trajectory.get(curTime);
    }
    Pose2d currentPose = m_pose.get();
    Pose2d poseRef = desiredState.poseMeters;
    // Calculate feedforward velocities (field-relative).
    double xFF = desiredState.vxMetersPerSecond;
    double yFF = desiredState.vyMetersPerSecond;
    double thetaFF =
        m_thetaController.calculate(m_pose.get().getRotation().getRadians(), 
          desiredState.poseMeters.getRotation().getRadians());

    SmartDashboard.putString("trajPose", poseRef.toString());
    // Calculate feedback velocities (based on position error).
    double xFeedback = m_xController.calculate(currentPose.getX(), poseRef.getX());
    double yFeedback = m_yController.calculate(currentPose.getY(), poseRef.getY());

    // Return next output.
    m_outputSpeeds.accept(ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed((m_trajectory.size()-1) * 0.02);
  }
}
