package frc.robot.subsystems.Shooter.Hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    
    @AutoLog
    public static class HoodIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public boolean forwardLimitReached = false;
        public boolean reverseLimitReached = false;
    }

    public default void updateInputs(HoodIOInputs inputs) {}
    public default void setAngle(double rad) {}
    public default void setVoltage(double volts) {}
    public default void stop() {}
}
