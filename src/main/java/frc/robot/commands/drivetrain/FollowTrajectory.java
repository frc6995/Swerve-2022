package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivebaseS;

public class FollowTrajectory extends SwerveControllerCommand {

    /**
     * Command to follow a given Trajectory using the SwerveControllerCommand class, which in turn uses HolonomicDriveController
     */

    public FollowTrajectory(Trajectory trajectory, DrivebaseS drive) {

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
            DriveConstants.m_kinematics, 
            new PIDController(0.001, 0, 0),
            new PIDController(0.001, 0, 00), 
            drive.rotationController,
            drive::setModuleStates, 
            drive
        );    
    }
}