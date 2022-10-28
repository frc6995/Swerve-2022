package frc.robot.util.sim;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;

public class SparkMaxEncoderWrapper {
    private static final int deviceManufacturer = 5; // REV
    private static final int deviceType = 2; // Spark Max
    private static final int apiId = 98; // Periodic status 2
  
    private final CANSparkMax sparkMax;
    private final CAN canInterface;
    private final LinearFilter velocityFilter;
    private final Notifier notifier;
  
    private boolean firstCycle = true;
    private double timestamp = 0.0;
    private double position = 0.0;
    private double velocity = 0.0;
    private double simPosition = 0.0;
    private double simVelocity = 0.0;
    private double positionConversionFactor = 0.0;
    private double velocityConversionFactor = 0.0;
  
    /**
     * Creates a new SparkMaxDerivedVelocityController using a default set of parameters.
     */
    public SparkMaxEncoderWrapper(CANSparkMax sparkMax) {
      this(sparkMax, 0.02, 5);
    }
  
    /** Creates a new SparkMaxDerivedVelocityController. */
    public SparkMaxEncoderWrapper(CANSparkMax sparkMax,
        double periodSeconds, int averagingTaps) {
            this.sparkMax = sparkMax;
            velocityFilter = LinearFilter.movingAverage(averagingTaps);
            
            positionConversionFactor = sparkMax.getEncoder().getPositionConversionFactor();
            velocityConversionFactor = sparkMax.getEncoder().getVelocityConversionFactor();
            sparkMax.getEncoder().setPositionConversionFactor(1.0);
            int periodMs = (int) (periodSeconds * 1000);
            sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, periodMs);
        
            canInterface =
                new CAN(sparkMax.getDeviceId(), deviceManufacturer, deviceType);
            
            notifier = new Notifier(this::update);
            notifier.startPeriodic(periodSeconds);

    }
  
    /**
     * Reads new data, updates the velocity measurement, and runs the controller.
     */
    private void update() {
      CANData canData = new CANData();
      boolean isFresh = canInterface.readPacketNew(apiId, canData);
      double newTimestamp = canData.timestamp / 1000.0;
      double newPosition = ByteBuffer.wrap(canData.data)
          .order(ByteOrder.LITTLE_ENDIAN).asFloatBuffer().get(0);
  
      if (isFresh) {
        synchronized (this) {
          if (!firstCycle) {
            velocity = velocityFilter.calculate(
                (newPosition - position) / (newTimestamp - timestamp) * 60);
          }
          firstCycle = false;
          timestamp = newTimestamp;
          position = newPosition;

        }
      }
    }
  
    /**
     * Returns the current position in rotations.
     */
    public synchronized double getPosition() {
        if(RobotBase.isReal()) {
            return position * positionConversionFactor;
        }
        else {
            return simPosition;
        }
      
    }
  
    /**
     * Returns the current velocity in rotations/minute.
     */
    public synchronized double getVelocity() {
        if(RobotBase.isReal()) {
            return velocity * velocityConversionFactor;
        } else {
            return simVelocity;
        }
    }

    public synchronized void setPosition(double position) {
        // we still want the encoder to report in motor shaft rotations, so divide by conversion factor.
        sparkMax.getEncoder().setPosition(position / positionConversionFactor);
        setSimPosition(position);
    }

    public synchronized void setSimPosition(double position) {
        simPosition = position;
    }

    public synchronized void setSimVelocity(double velocity) {
        simVelocity = velocity;
    }

    
  }
