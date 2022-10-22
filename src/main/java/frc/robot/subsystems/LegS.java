package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.command.RunEndCommand;

public class LegS extends SubsystemBase {
    private Solenoid solenoid;
      
      /**
      * Constructs a new LegS
      */
      public LegS(int solenoidCANID) {
      solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, solenoidCANID);
    }
      
      public void kickThoseFeet() {
          solenoid.set(true);
      }
      public void relax() {
          solenoid.set(false);
      }	
  
      public Command legKickC() {
          return new RunEndCommand(this::kickThoseFeet, this::relax, this);
      }
  }
