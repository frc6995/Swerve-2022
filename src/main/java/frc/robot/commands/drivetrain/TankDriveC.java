package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivebaseS;

public class TankDriveC extends CommandBase {

    /**
     * Command to allow for driver input in teleop
     * Can't be inlined efficiently if we want to edit the inputs in any way (deadband, square, etc.)
     */

    private final DrivebaseS drive;

    /**
     * Joysticks return DoubleSuppliers when the get methods are called
     * This is so that joystick getter methods can be passed in as a parameter but will continuously update, 
     * versus using a double which would only update when the constructor is called
     */
    private final DoubleSupplier forwardX;
    private final DoubleSupplier rotation;
    private final DifferentialDriveKinematics diffDriveKinematics = new DifferentialDriveKinematics(DriveConstants.WHEEL_BASE_WIDTH_M);


    public TankDriveC(
        DoubleSupplier fwdX, 
        DoubleSupplier rot,
        DrivebaseS subsystem
    ) {

        drive = subsystem;
        forwardX = fwdX;
        rotation = rot;

        addRequirements(subsystem);

    }
    
    @Override
    public void execute() {

        /**
         * Units are given in meters per second radians per second
         * Since joysticks give output from -1 to 1, we multiply the outputs by the max speed
         * Otherwise, our max speed would be 1 meter per second and 1 radian per second
         */
        double fwdBack = forwardX.getAsDouble();
        double turn = rotation.getAsDouble();

        fwdBack = MathUtil.applyDeadband(fwdBack, 0.02);
        turn = MathUtil.applyDeadband(turn, 0.05);
        fwdBack = fwdBack * 6.5 / 12;

        boolean quickTurn = false;
        if (fwdBack == 0) {
          quickTurn = true;
          turn *= 0.15;
        }
        else {
          turn *= 0.1;
        }
        WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(fwdBack, turn, quickTurn);
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
            speeds.left * 5,
            speeds.right * 5);
        ChassisSpeeds chassisSpeeds = diffDriveKinematics.toChassisSpeeds(wheelSpeeds);
        drive.drive(chassisSpeeds);


    }

    // method to deadband inputs to eliminate tiny unwanted values from the joysticks
    public double deadbandInputs(double input) {

        if (Math.abs(input) < 0.07) return 0.0;
        return input;

    }

}