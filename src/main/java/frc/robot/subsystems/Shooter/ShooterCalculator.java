//Inspired by and adapted from FRC Team 6328

package frc.robot.subsystems.Shooter;

import frc.robot.util.AllianceFlipUtil;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.siteConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class ShooterCalculator {
        private final CommandSwerveDrivetrain drive;
        private final InterpolatingTreeMap<Double, Rotation2d> hoodMap;
        private final InterpolatingTreeMap<Double, AngularVelocity> rollMap;
        private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
        private final double phaseDelay = 0.03;
        public static Transform3d robotToTurret = new Transform3d(0.2, 0.0, 0.44, Rotation3d.kZero);

        private static final double HARD_MIN_RADS = Units.degreesToRadians(-210.0);
        private static final double HARD_MAX_RADS = Units.degreesToRadians(210.0);

        private static final double SOFT_MIN_RADS = Units.degreesToRadians(-190.0);
        private static final double SOFT_MAX_RADS = Units.degreesToRadians(190.0);

        private double lastSetpointRads = 0.0;

        // 定義狀態列舉
        public enum ShootState {
                ACTIVE_SHOOTING, // 準備開火：使用全範圍 (Hard Limits)
                TRACKING // 純追蹤/待機：使用縮小範圍 (Soft Limits)
        }

        public ShooterCalculator(CommandSwerveDrivetrain drive) {
                this.drive = drive;
                hoodMap = new InterpolatingTreeMap<>(
                                InverseInterpolator.forDouble(),
                                Rotation2d::interpolate);
                rollMap = new InterpolatingTreeMap<>(
                                InverseInterpolator.forDouble(),
                                (start, end, t) -> {
                                        // 邏輯：拆成 double (RPM) -> 算數學 -> 包回 Unit
                                        double startVal = start.in(RotationsPerSecond);
                                        double endVal = end.in(RotationsPerSecond);
                                        double interpolated = MathUtil.interpolate(startVal, endVal, t);
                                        return RotationsPerSecond.of(interpolated);
                                });
                hoodMap.put(2.0, Rotation2d.fromDegrees(15.0));

                rollMap.put(2.0, RotationsPerSecond.of(3000));

                timeOfFlightMap.put(5.68, 1.16);
                timeOfFlightMap.put(4.55, 1.12);
                timeOfFlightMap.put(3.15, 1.11);
                timeOfFlightMap.put(1.88, 1.09);
                timeOfFlightMap.put(1.38, 0.90);
        }

        public record ShootingState(
                        Rotation2d turretFieldAngle, // 砲塔該瞄準的「場地角度」
                        double effectiveDistance // 用來查表的「有效距離」
        ) {
        }

        public ShootingState calculateShootingState() {
                // 1. 取得基本狀態
                Pose2d estimatedPose = drive.getPose2d();
                ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();

                // 2. 延遲補償 (Phase Delay Compensation)
                // 預測 "現在命令發出後，實際執行時" 機器人會在哪
                estimatedPose = estimatedPose.exp(new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

                // 3. 計算砲塔位置
                Pose2d turretPosition = estimatedPose.transformBy(
                                new Transform2d(
                                                robotToTurret.getTranslation().toTranslation2d(),
                                                robotToTurret.getRotation().toRotation2d()));

                // 4. 計算目標位置 (處理紅藍翻轉)
                Translation2d target = AllianceFlipUtil.apply(siteConstants.topCenterPoint.toTranslation2d());
                double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

                // 5. 計算砲塔的場地速度 (Turret Field Velocity)
                ChassisSpeeds robotVelocity = drive.getFieldVelocity();
                double robotAngle = estimatedPose.getRotation().getRadians();

                // V_turret = V_robot + (Omega x Radius)
                // 這是為了算出機器人旋轉時，砲塔本身被甩動的速度
                double turretVelocityX = robotVelocity.vxMetersPerSecond
                                + robotVelocity.omegaRadiansPerSecond
                                                * (robotToTurret.getY() * Math.cos(robotAngle)
                                                                - robotToTurret.getX() * Math.sin(robotAngle));
                double turretVelocityY = robotVelocity.vyMetersPerSecond
                                + robotVelocity.omegaRadiansPerSecond
                                                * (robotToTurret.getX() * Math.cos(robotAngle)
                                                                - robotToTurret.getY() * Math.sin(robotAngle));

                // 6. 核心迭代運算 (Iterative Solver)
                // 找出 "Lookahead Pose" (虛擬發射點)
                double timeOfFlight = 0.0;
                Pose2d lookaheadPose = turretPosition;
                double lookaheadTurretToTargetDistance = turretToTargetDistance;

                // 跑 5 次迭代通常就足夠收斂了，不用跑到 20 次
                for (int i = 0; i < 5; i++) {
                        // 查表：根據目前的預測距離，查子彈飛多久
                        timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);

                        // 計算偏移量：在飛行時間內，機器人速度會把球帶偏多少
                        double offsetX = turretVelocityX * timeOfFlight;
                        double offsetY = turretVelocityY * timeOfFlight;

                        // 更新 Lookahead Pose
                        lookaheadPose = new Pose2d(
                                        turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                                        turretPosition.getRotation());

                        // 更新距離
                        lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
                }

                // ==========================================
                // 7. (新增) 計算最終瞄準角度
                // ==========================================

                // 數學原理：我們假設機器人已經瞬間移動到了 lookaheadPose
                // 所以直接計算 "LookaheadPose -> Target" 的向量角度，就是正確的瞄準角
                Translation2d vectorToTarget = target.minus(lookaheadPose.getTranslation());
                Rotation2d targetFieldAngle = vectorToTarget.getAngle();

                Pose2d simturretPosition = estimatedPose.transformBy(
                                new Transform2d(
                                                robotToTurret.getTranslation().toTranslation2d(),
                                                robotToTurret.getRotation().toRotation2d()));
                
                Logger.recordOutput("simturretPosition", new Pose2d(simturretPosition.getX(),simturretPosition.getY(),targetFieldAngle));
                Logger.recordOutput("Shooting/LookaheadPose", lookaheadPose);

                return new ShootingState(targetFieldAngle, lookaheadTurretToTargetDistance);
        }

        public Angle TurretCalculate(Rotation2d robotHeading, Angle targetRad, ShootState state) {

                // 1. 根據狀態決定目前的 "合法範圍"
                double currentMin, currentMax;

                if (state == ShootState.ACTIVE_SHOOTING) {
                        // 射擊時：火力全開，使用硬體極限
                        currentMin = HARD_MIN_RADS;
                        currentMax = HARD_MAX_RADS;
                } else {
                        // 追蹤時：保守一點，使用軟體限制
                        currentMin = SOFT_MIN_RADS;
                        currentMax = SOFT_MAX_RADS;
                }

                // 2. 算出 "基礎" 相對角度

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
                        return TurretCalculate(robotHeading, targetRad, ShootState.ACTIVE_SHOOTING);
                }

                if (!foundValidAngle) {
                        // 真的完全轉不到 (超出硬體極限)
                        return Radians.of(lastSetpointRads);
                }

                lastSetpointRads = bestAngle;
                return Radians.of(bestAngle);
        }
}