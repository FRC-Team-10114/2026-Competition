package frc.robot.subsystems.Shooter.Hood;

import edu.wpi.first.units.measure.Angle;

public interface HoodIO {

    public void setAngle(Angle angle);

    public double getAngle();
    
    public void CANcoderConfig();

    public void configure();

    public boolean isAtSetPosition();
}
