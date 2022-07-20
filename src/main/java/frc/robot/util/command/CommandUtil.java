package frc.robot.util.command;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CommandUtil {
    public static Command errorC(Command attemptCommand, Command errorCommand, BooleanSupplier errorCondition) {
        return new ConditionalCommand(
          errorCommand,
          new SequentialCommandGroup(
            attemptCommand.until(errorCondition),
            new ConditionalCommand(
              new ScheduleCommand(errorCommand),
              new InstantCommand(),
              errorCondition
            )
          ),
          errorCondition
        );
      }
}
