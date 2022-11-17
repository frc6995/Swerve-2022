package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivebaseS;

public class OperatorControlC extends CommandBase {

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
    private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(3);
    private final DoubleSupplier forwardY;
    private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(3);
    private final DoubleSupplier rotation;
    private final SlewRateLimiter thetaRateLimiter = new SlewRateLimiter(3);
    
    private final boolean isFieldRelative;

    public OperatorControlC(
        DoubleSupplier fwdX, 
        DoubleSupplier fwdY, 
        DoubleSupplier rot,
        DrivebaseS subsystem
    ) {

        drive = subsystem;
        forwardX = fwdX;
        forwardY = fwdY;
        rotation = rot;

        isFieldRelative = true;

        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        xRateLimiter.reset(0);
        yRateLimiter.reset(0);
        thetaRateLimiter.reset(0);
        drive.resetThetaProfile(drive.getPoseHeading());
    }
    
    @Override
    public void execute() {

        /**
         * Units are given in meters per second radians per second
         * Since joysticks give output from -1 to 1, we multiply the outputs by the max speed
         * Otherwise, our max speed would be 1 meter per second and 1 radian per second
         */

        double fwdX = -forwardX.getAsDouble();
        fwdX = Math.copySign(fwdX, fwdX);
        fwdX = deadbandInputs(fwdX);
        fwdX = xRateLimiter.calculate(fwdX);
        fwdX *= Units.feetToMeters(8);

        double fwdY = -forwardY.getAsDouble();
        fwdY = Math.copySign(fwdY, fwdY);
        fwdY = deadbandInputs(fwdY);
        fwdY = yRateLimiter.calculate(fwdY);
        fwdY *= Units.feetToMeters(8);

        double rot = -rotation.getAsDouble();
        //rot = Math.copySign(rot * rot, rot);
        rot = deadbandInputs(rot);
        rot = thetaRateLimiter.calculate(rot);
        rot *= Units.degreesToRadians(DriveConstants.teleopTurnRateDegPerSec);

        drive.drive(
            fwdX,
            fwdY,
            rot,
            isFieldRelative
        );
    }

    // method to deadband inputs to eliminate tiny unwanted values from the joysticks
    public double deadbandInputs(double input) {

        if (Math.abs(input) < 0.07) return 0.0;
        return input;

    }

}