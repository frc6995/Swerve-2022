package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    
    private RobotContainer robotContainer;

    private Command autonomousCommand;

    @Override
    public void robotInit() {

        robotContainer = new RobotContainer();
        NetworkTableInstance.getDefault().setUpdateRate(.01);
    }

    @Override
    public void robotPeriodic() {

        CommandScheduler.getInstance().run();
        robotContainer.periodic();
        
    }

    @Override
    public void autonomousInit() {
        
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) autonomousCommand.schedule();

    }

    @Override
    public void teleopInit() {

        if (autonomousCommand != null) autonomousCommand.cancel();

    }

    @Override
    public void disabledPeriodic() {
        // TODO Auto-generated method stub
        super.disabledPeriodic();
        CommandScheduler.getInstance().cancelAll();
    }

}
