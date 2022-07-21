package frc.robot.commands.drivetrain;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.util.TrajectoryReader;
import frc.robot.util.trajectory.PoseListSwerveControllerCommand;

public class FollowPoseList extends PoseListSwerveControllerCommand {

    /**
     * Command to follow a given Trajectory using the SwerveControllerCommand class, which in turn uses HolonomicDriveController
     */

    public FollowPoseList(List<TrajectoryReader.State> trajectory, DrivebaseS drive) {

        /**
         * Super constructor for SwerveControllerCommand
         * Parameters: 
         * trajectory to be followed
         * method reference to the pose supplier
         * kinematics of the drive (wheel placements on robot)
         * x controller
         * y controller
         * rotation controller
         * method reference to the module control method
         * requirements (drive subsystem)
         */   
        super(
            trajectory, 
            drive::getPose,
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0), 
            drive.rotationController,
            drive::drive, 
            drive
        );  

    }
}