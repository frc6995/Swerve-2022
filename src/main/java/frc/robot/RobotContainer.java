package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.apriltag.AprilTag;
import edu.wpi.first.wpilibj.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.InputDevices;
import frc.robot.auto.Trajectories;
import frc.robot.commands.drivetrain.OperatorControlC;
import frc.robot.subsystems.DrivebaseS;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.util.NomadMathUtil;

public class RobotContainer {

    /**
     * Establishes the controls and subsystems of the robot
     */

    private final CommandXboxController gamepad = new CommandXboxController(InputDevices.GAMEPAD_PORT);
    @Log
    private final DrivebaseS drivebaseS = new DrivebaseS();
    private AprilTagFieldLayout aprilTagFieldLayout; 

    @Log
    private final Field2d field = new Field2d();

    PhotonCamera camera;
    
    @Log
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    PathPlannerTrajectory pathPlannerTrajectory;

    public RobotContainer() {

        field.getObject("target").setPose(new Pose2d(4, 4, new Rotation2d()));
        Pose2d targetObject = field.getObject("target").getPose();

        // INIT PHOTONVISION
        camera = new PhotonCamera("NexiGo_N60_FHD_Webcam");
        PhotonCamera.setVersionCheckEnabled(false);

        // INIT APRILTAGS
        var fieldLayoutPath = Filesystem.getDeployDirectory().getAbsolutePath() + "\\apriltag\\layout-2022-formatted.json";
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(fieldLayoutPath);
        }
        catch (IOException e) {
            DriverStation.reportWarning("AprilTag Layout not loaded! " + fieldLayoutPath, e.getStackTrace());
            aprilTagFieldLayout = new AprilTagFieldLayout(List.of(), 0, 0);
        }
        aprilTagFieldLayout.setAlliance(DriverStation.getAlliance());
        aprilTagFieldLayout.getTags().forEach((tag)->{
            field.getObject("tag" + tag.ID).setPose(
                tag.pose.toPose2d());
        });

        pathPlannerTrajectory = PathPlanner.generatePath(
            new PathConstraints(4, 4), 
            new PathPoint(
                drivebaseS.getPose().getTranslation(),
                NomadMathUtil.getDirection(new Transform2d(drivebaseS.getPose(), targetObject)),
                drivebaseS.getPoseHeading()), // position, heading
            new PathPoint(targetObject.getTranslation(), targetObject.getRotation()) // position, heading
                );      
        gamepad.a().toggleWhenActive(drivebaseS.chasePoseC(field.getObject("target")::getPose, field));
        drivebaseS.setDefaultCommand(
            new OperatorControlC(
                gamepad::getLeftY,
                gamepad::getLeftX,
                gamepad::getRightX,
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
        field.getObject("pathOTF").setTrajectory(pathPlannerTrajectory);
        autoSelector.setDefaultOption("pathPlanner", drivebaseS.chasePoseC(()->new Pose2d(), field)
           
        );
        
        //field.getObject("pathPlanner").setTrajectory((Trajectory) pathPlannerTrajectory);
    }

    public void configureButtonBindings() {
        new Trigger(RobotController::getUserButton).whenActive(()->drivebaseS.resetPose(new Pose2d()));
        gamepad.pov.center().whenInactive(
            new InstantCommand(
                ()->drivebaseS.setRotationState(
                    new TrapezoidProfile.State(Units.degreesToRadians(gamepad.getPOV()), 0)
                )
            )
        );
        // gamepad.b().toggleWhenActive( new RunCommand(
        //     ()->drivebaseS.setRotationState(
        //         new TrapezoidProfile.State(
        //             NomadMathUtil.getDirection(
        //                 drivebaseS.getPose(),
        //                 Trajectories.HUB_CENTER_POSE
        //             ).getRadians(), 0)
        //     )
        // ));
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }

    public void periodic() {
    //     PhotonPipelineResult result = camera.getLatestResult();
    //     Pose2d pose;
    // if(result.hasTargets()) {
    //   pose = new Pose3d().transformBy(result.getBestTarget().getCameraToTarget())
    //     .toPose2d().transformBy(new Transform2d(new Translation2d(),new Rotation2d(-Math.PI/2)));
    //   field.getObject("target").setPose(pose);
    //   SmartDashboard.putNumber("ambig", result.getBestTarget().getPoseAmbiguity());
    // }
    // else {
    //     field.getObject("target").setPose(field.getRobotPose());
    // }
        
field.getObject("pathOTF").setTrajectory(pathPlannerTrajectory);
        drivebaseS.drawRobotOnField(field);


    }

    public void onEnabled(){
        drivebaseS.resetRelativeRotationEncoders();
    }
}
