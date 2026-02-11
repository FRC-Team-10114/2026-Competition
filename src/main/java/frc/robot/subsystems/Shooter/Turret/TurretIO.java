package frc.robot.subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.Shooter.ShooterConstants;

public abstract class TurretIO {

    public enum ShootState {
        ACTIVE_SHOOTING, // 準備開火：使用全範圍 (Hard Limits)
        TRACKING // 純追蹤/待機：使用縮小範圍 (Soft Limits)
    }

    public double lastSetpointRads = 0.0;

    public abstract void setAngle(Rotation2d robotHeading, Angle targetRad, ShootState state);

    public abstract void resetAngle();

    public abstract Angle getAngle();

    public abstract boolean isAtSetPosition();

    public Angle Calculate(Rotation2d robotHeading, Angle targetRad, ShootState state) {

        // 1. 根據狀態決定目前的 "合法範圍"
        double currentMin, currentMax;

        if (state == ShootState.ACTIVE_SHOOTING) {
            // 射擊時：火力全開，使用硬體極限
            currentMin = ShooterConstants.HARD_MIN_RADS;
            currentMax = ShooterConstants.HARD_MAX_RADS;
        } else {
            // 追蹤時：保守一點，使用軟體限制
            currentMin = ShooterConstants.SOFT_MIN_RADS;
            currentMax = ShooterConstants.SOFT_MAX_RADS;
        }

        // 1. 先把單位 (Measure) 轉成幾何 (Rotation2d)
        Rotation2d targetRotation = Rotation2d.fromRadians(targetRad.in(Radians));

        // 2. 現在兩個都是 Rotation2d 了，可以相減
        Rotation2d relativeGoal = targetRotation.minus(robotHeading);

        // 3. 取出弧度數值
        double baseAngleRads = relativeGoal.getRadians();

        // 3. 窮舉候選角度 (解繞核心)
        double bestAngle = 0.0;
        boolean foundValidAngle = false;

        for (int i = -2; i <= 2; i++) {
            double candidate = baseAngleRads + (Math.PI * 2.0 * i);

            // 使用動態決定的 currentMin/Max 來過濾
            if (candidate >= currentMin && candidate <= currentMax) {

                if (!foundValidAngle) {
                    bestAngle = candidate;
                    foundValidAngle = true;
                } else {
                    // 如果有多個合法角度，選離上次目標最近的
                    if (Math.abs(lastSetpointRads - candidate) < Math
                            .abs(lastSetpointRads - bestAngle)) {
                        bestAngle = candidate;
                    }
                }
            }
        }

        // 防呆：如果在 Soft Limit 找不到角度 (極罕見)，試著放寬到 Hard Limit 找一次
        // 避免因為 Soft Limit 設太窄導致追蹤丟失
        if (!foundValidAngle && state == ShootState.TRACKING) {
            return Calculate(robotHeading, targetRad, ShootState.ACTIVE_SHOOTING);
        }

        if (!foundValidAngle) {
            // 真的完全轉不到 (超出硬體極限)
            return Radians.of(lastSetpointRads);
        }

        lastSetpointRads = bestAngle;
        return Radians.of(bestAngle);
    }
}
