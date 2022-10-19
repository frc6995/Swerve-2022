package frc.robot;

import java.io.File;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.drivetrain.FollowPoseList;
import frc.robot.commands.drivetrain.OperatorControlC;
import frc.robot.commands.drivetrain.TankDriveC;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.util.trajectory.TrajectoryReader;
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
            //new RunCommand(()->{drivebaseS.driveToPose(field.getObject("target").getPose());}, drivebaseS)
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

        
        //field.getObject("cupid").setPoses(poseList);

    }

    public void configureButtonBindings() {
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
