package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;

public class Robot extends TimedRobot {
    
    private RobotContainer robotContainer;

    private Command autonomousCommand;

    @Override
    public void robotInit() {

        robotContainer = new RobotContainer();
        Logger.configureLoggingAndConfig(robotContainer, false);
        NetworkTableInstance.getDefault().setUpdateRate(.01);
    }

    @Override
    public void robotPeriodic() {

        CommandScheduler.getInstance().run();
        robotContainer.periodic();
        Logger.updateEntries();
        
    }

    @Override
    public void autonomousInit() {
        robotContainer.onEnabled();
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) autonomousCommand.schedule();

    }

    @Override
    public void teleopInit() {
        robotContainer.onEnabled();
        if (autonomousCommand != null) autonomousCommand.cancel();

    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
        CommandScheduler.getInstance().cancelAll();
    }

}
