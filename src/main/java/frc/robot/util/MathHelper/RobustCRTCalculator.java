package frc.robot.util.MathHelper;

public class RobustCRTCalculator {

    public static double calculateAbsolutePosition(
        EncoderWithGearRatio master, 
        EncoderWithGearRatio slave
        ) {

        // 1. 計算相位差
        double diff = master.position - slave.position;
        
        // 2. 處理 Wrapping 確保在 [0, 1]
        diff %= 1.0;
        if (diff < 0) diff += 1.0;

        // 3. 估算當前 master 轉到了第幾圈 (k)
        // 使用 round 抵消微小雜訊
        int k = (int) Math.round(diff * slave.gearRatio);

        // 4. 重構總體絕對位置 (相對於大週期的比例)
        double absolutePos = (k + master.position) / master.gearRatio;

        // 5. 邊界修正
        if (absolutePos >= 1.0) absolutePos -= 1.0;
        else if (absolutePos < 0) absolutePos += 1.0;

        return absolutePos;
    }
}
