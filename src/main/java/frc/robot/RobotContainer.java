package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.InputDevices;
import frc.robot.auto.Trajectories;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.drivetrain.OperatorControlC;
import frc.robot.subsystems.DrivebaseS;

public class RobotContainer {

    /**
     * Establishes the controls and subsystems of the robot
     */

    private final CommandXboxController gamepad = new CommandXboxController(InputDevices.gamepadPort);

    private final DrivebaseS drivebaseS = new DrivebaseS();

    private final Field2d field = new Field2d();
    
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public RobotContainer() {

        drivebaseS.setDefaultCommand(
            new OperatorControlC(
                gamepad::getLeftY,
                gamepad::getLeftX,
                gamepad::getRightX,
                true,
                drivebaseS
            )
        );

        configureButtonBindings();

        autoSelector.setDefaultOption("mid to ring", 
            new InstantCommand(
                ()->drivebaseS.resetPose(Trajectories.MID_BALL_START_POSE))
            .andThen(
                new FollowTrajectory(Trajectories.MID_START_TO_MID_RING, drivebaseS)
            )
            );
        SmartDashboard.putData(field);

    }

    public void configureButtonBindings() {
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }

    public void periodic() {
        drivebaseS.drawRobotOnField(field);
    }

}
