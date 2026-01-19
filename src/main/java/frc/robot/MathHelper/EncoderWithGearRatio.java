package frc.robot.MathHelper;

public class EncoderWithGearRatio {
    
    public final double position;
    public final int gearRatio;

/**
     * @param position  當前編碼器位置
     * @param gearRatio 物理齒數
     */
    public EncoderWithGearRatio(double position, int gearRatio) {
        this.position = position;
        this.gearRatio = gearRatio;
    }
}
