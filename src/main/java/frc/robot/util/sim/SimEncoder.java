package frc.robot.util.sim;

/** Add your docs here. */
public class SimEncoder {
    private double position = 0;
    private double velocity = 0;

    public SimEncoder() {}

    public void setPosition(double position) {
        this.position = position;
    }

    public double getPosition() {
        return position;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double getVelocity() {
        return velocity;
    }
}
