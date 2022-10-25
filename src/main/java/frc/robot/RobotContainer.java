package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.drivetrain.OperatorControlC;
import frc.robot.subsystems.DrivebaseS;
import io.github.oblarg.oblog.annotations.Log;

public class RobotContainer {

    /**
     * Establishes the controls and subsystems of the robot
     */

    private final CommandXboxController gamepad = new CommandXboxController(InputDevices.GAMEPAD_PORT);
    @Log
    private final DrivebaseS drivebaseS = new DrivebaseS();


    @Log
    private final Field2d field = new Field2d();
    
    @Log
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public RobotContainer() {

        field.getObject("target").setPose(new Pose2d());

        drivebaseS.setDefaultCommand(
            new OperatorControlC(
                gamepad::getLeftY,
                gamepad::getLeftX,
                gamepad::getRightX,
                true,
                drivebaseS
            )
            // new InstantCommand(drivebaseS::resetPID).andThen(
            // new RunCommand(()->{drivebaseS.driveToPose(field.getObject("target").getPose());}, drivebaseS)
            // )
            // new TankDriveC(
            //     gamepad::getLeftY,
            //     gamepad::getLeftX,
            //     drivebaseS
            // )
        );

        configureButtonBindings();

        PathPlannerTrajectory pathPlannerTrajectory = PathPlanner.loadPath("path1", 4, 4, false);
        autoSelector.setDefaultOption("pathPlanner", 
            new InstantCommand(
                ()->{ 
                    Pose2d initialPose = pathPlannerTrajectory.getInitialHolonomicPose();
                    drivebaseS.resetPose(initialPose);
                })
            .andThen(
                drivebaseS.pathPlannerCommand(pathPlannerTrajectory)
            )
        );
        
        field.getObject("pathPlanner").setTrajectory((Trajectory) pathPlannerTrajectory);
    }

    public void configureButtonBindings() {
        new Trigger(RobotController::getUserButton).whenActive(()->drivebaseS.resetPose(new Pose2d()));
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }

    public void periodic() {
        drivebaseS.drawRobotOnField(field);
    }

    public void onEnabled(){
        drivebaseS.resetRelativeRotationEncoders();
    }
}
