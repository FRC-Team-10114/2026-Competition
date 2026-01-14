package frc.robot.subsystems.Shooter.Hood;


public interface HoodIO {
    
    // default 提供預設的實作方法，在實作尚未定義時使用預設，且不會報錯

    public default void setAngle(double rad) {}
    public default void setVoltage(double volts) {}
    public default void stop() {}
}
