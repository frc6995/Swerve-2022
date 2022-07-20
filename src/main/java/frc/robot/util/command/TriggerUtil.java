package frc.robot.util.command;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerUtil {
    public static Trigger rising(BooleanSupplier condition) {
        return new Trigger(
            new BooleanSupplier() {
                private boolean m_lastValue = condition.getAsBoolean();

                @Override
                public boolean getAsBoolean() {
                    boolean currentValue = condition.getAsBoolean();
                    boolean returnValue = (!m_lastValue) && currentValue;
                    m_lastValue = currentValue;
                    return returnValue;
                }
                
            }
        );
    }

    public static Trigger falling(BooleanSupplier condition) {
        return new Trigger(
            new BooleanSupplier() {
                private boolean m_lastValue = condition.getAsBoolean();

                @Override
                public boolean getAsBoolean() {
                    boolean currentValue = condition.getAsBoolean();
                    boolean returnValue = (m_lastValue) && !currentValue;
                    m_lastValue = currentValue;
                    return returnValue;
                }
                
            }
        );
    }
}
