package frc.robot.subsystems.Shooter.Hood;

import edu.wpi.first.units.measure.Angle;

public interface HoodIO {

    public void setAngle(Angle angle);
    public void setVoltage(double volts);
    public void stop();

    public double getAngle();

    public void reset();
}
