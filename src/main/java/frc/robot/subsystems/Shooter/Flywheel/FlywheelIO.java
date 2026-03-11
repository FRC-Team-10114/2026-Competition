package frc.robot.subsystems.Shooter.Flywheel;

import edu.wpi.first.units.measure.AngularVelocity;

public interface FlywheelIO {
    
    public void setVelocity(AngularVelocity RPS);

    public AngularVelocity getVelocity();

    public boolean isAtSetPosition();
}
