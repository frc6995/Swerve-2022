package frc.robot;

import java.io.File;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.drivetrain.FollowPoseList;
import frc.robot.commands.drivetrain.OperatorControlC;
import frc.robot.commands.drivetrain.TankDriveC;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.LegS;
import frc.robot.util.trajectory.TrajectoryReader;
import io.github.oblarg.oblog.annotations.Log;

public class RobotContainer {

    /**
     * Establishes the controls and subsystems of the robot
     */

    private final CommandXboxController gamepad = new CommandXboxController(InputDevices.GAMEPAD_PORT);
    @Log
    private final DrivebaseS drivebaseS = new DrivebaseS();

    private final LegS rightLeg = new LegS(0);
  private final LegS leftLeg = new LegS(1);

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
        File trajectoryFile = new File(
            Filesystem.getDeployDirectory().getPath() + "/trajectories/R1.csv");
        List<Pose2d> poseList =
            TrajectoryReader.readFilePoseList(trajectoryFile);
        
        autoSelector.setDefaultOption("mid to ring", 
            new InstantCommand(
                ()->drivebaseS.resetPose(poseList.get(0)))
            .andThen(
                new RepeatCommand(new FollowPoseList(
                    TrajectoryReader.readFileTrajectory(trajectoryFile), drivebaseS))
            )
        );
        new Trigger(RobotController::getUserButton).whenActive(()->drivebaseS.resetPose(poseList.get(0)));

        PathPlannerTrajectory pathPlannerTrajectory = PathPlanner.loadPath("path1", 4, 4, false);
        autoSelector.addOption("pathPlanner", 
            new InstantCommand(
                ()->{ 
                    Pose2d initialPose = new Pose2d(pathPlannerTrajectory.getInitialState().poseMeters.getTranslation(),
                    pathPlannerTrajectory.getInitialState().holonomicRotation);
                    drivebaseS.resetPose(initialPose);
                })
            .andThen(
                drivebaseS.pathPlannerCommand(pathPlannerTrajectory)
            )
        );
        
        field.getObject("pathPlanner").setTrajectory((Trajectory) pathPlannerTrajectory);
        field.getObject("cupid").setPoses(poseList);
        field.getObject("cupidStart").setPose(poseList.get(0));
        field.getObject("cupidNext").setPose(poseList.get(1));



    }

    public void configureButtonBindings() {
        new Trigger(RobotController::getUserButton).whenActive(()->drivebaseS.resetPose(new Pose2d()));
        gamepad.x().whileActiveOnce(leftLeg.legKickC());
        gamepad.b().whileActiveOnce(rightLeg.legKickC());
        gamepad.start().whenActive(()->getAutonomousCommand().schedule());
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
