/*
 * Original code from Littleton Robotics (Team 6328) - 2026 Season
 * Modified by Team [10114]
 * * Licensed under the MIT License.
 */

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.FieldConstants.siteConstants;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.FIeldHelper.AllianceFlipUtil;
import frc.robot.util.RobotStatus.RobotStatus;

public class ShooterCalculator {
        private final RobotStatus robotStatus;
        private final CommandSwerveDrivetrain drive;
        private final InterpolatingTreeMap<Double, Angle> hoodMap;
        private final InterpolatingTreeMap<Double, AngularVelocity> rollMap;
        private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
        private final double time_error = 0.0;
        private final double phaseDelay = 0.03 + time_error;
        public static Transform3d robotToTurret = new Transform3d(0.2, 0.0, 0.44, Rotation3d.kZero);


        private static final Angle Hood_MAX_RADS = Radians.of(ShooterConstants.Hood_MAX_RADS);


        public ShooterCalculator(CommandSwerveDrivetrain drive, RobotStatus robotStatus) {
                this.drive = drive;
                this.robotStatus = robotStatus;

                hoodMap = new InterpolatingTreeMap<>(
                                InverseInterpolator.forDouble(),
                                (start, end, t) -> {
                                        // 1. 把單位轉成 double (用 Radians 或 Degrees 都可以，統一就好)
                                        double startVal = start.in(Radians);
                                        double endVal = end.in(Radians);

                                        // 2. 算數學插值 (start + (end - start) * t)
                                        double result = MathUtil.interpolate(startVal, endVal, t);

                                        // 3. 把 double 包回 Angle 物件
                                        return Radians.of(result);
                                });
                rollMap = new InterpolatingTreeMap<>(
                                InverseInterpolator.forDouble(),
                                (start, end, t) -> {
                                        // 邏輯：拆成 double (RPM) -> 算數學 -> 包回 Unit
                                        double startVal = start.in(RotationsPerSecond);
                                        double endVal = end.in(RotationsPerSecond);
                                        double interpolated = MathUtil.interpolate(startVal, endVal, t);
                                        return RotationsPerSecond.of(interpolated);
                                });
                hoodMap.put(2.0, Radians.of(15.0));

                rollMap.put(2.0, RotationsPerSecond.of(10));

                timeOfFlightMap.put(5.68, 1.16);
                timeOfFlightMap.put(4.55, 1.12);
                timeOfFlightMap.put(3.15, 1.11);
                timeOfFlightMap.put(1.88, 1.09);
                timeOfFlightMap.put(1.38, 0.90);
        }

        public record ShootingState(
                        Rotation2d turretFieldAngle, // 砲塔該瞄準的「場地角度」
                        Angle HoopAngle, // 用來查表的「有效距離
                        AngularVelocity FlywheelRPS
        ) {
        }

        public ShootingState calculateShootingToHub() {
                Pose2d estimatedPose = drive.getPose2d();
                ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();

                estimatedPose = estimatedPose.exp(new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

                Pose2d turretPosition = estimatedPose.transformBy(
                                new Transform2d(
                                                robotToTurret.getTranslation().toTranslation2d(),
                                                robotToTurret.getRotation().toRotation2d()));

                Translation2d target = AllianceFlipUtil.apply(siteConstants.topCenterPoint.toTranslation2d());
                double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

                ChassisSpeeds robotVelocity = drive.getFieldVelocity();
                double robotAngle = estimatedPose.getRotation().getRadians();

                double turretVelocityX = robotVelocity.vxMetersPerSecond
                                + robotVelocity.omegaRadiansPerSecond
                                                * (robotToTurret.getY() * Math.cos(robotAngle)
                                                                - robotToTurret.getX() * Math.sin(robotAngle));
                double turretVelocityY = robotVelocity.vyMetersPerSecond
                                + robotVelocity.omegaRadiansPerSecond
                                                * (robotToTurret.getX() * Math.cos(robotAngle)
                                                                - robotToTurret.getY() * Math.sin(robotAngle));

                double timeOfFlight = 0.0;
                Pose2d lookaheadPose = turretPosition;
                double lookaheadTurretToTargetDistance = turretToTargetDistance;

                for (int i = 0; i < 5; i++) {
                        timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);

                        double offsetX = turretVelocityX * timeOfFlight;
                        double offsetY = turretVelocityY * timeOfFlight;

                        lookaheadPose = new Pose2d(
                                        turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                                        turretPosition.getRotation());

                        lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
                }

                Translation2d vectorToTarget = target.minus(lookaheadPose.getTranslation());
                Rotation2d targetFieldAngle = vectorToTarget.getAngle();

                Pose2d simturretPosition = estimatedPose.transformBy(
                                new Transform2d(
                                                robotToTurret.getTranslation().toTranslation2d(),
                                                robotToTurret.getRotation().toRotation2d()));
                Logger.recordOutput("ToHubsimturretPosition",
                                new Pose2d(simturretPosition.getX(), simturretPosition.getY(), targetFieldAngle));

                return new ShootingState(targetFieldAngle, hoodMap.get(lookaheadTurretToTargetDistance),rollMap.get(lookaheadTurretToTargetDistance));
        }

        // -------------------------------------------------------------------------------------------------------------------
        public ShootingState calculateShootingToAlliance() {
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
                Translation2d target;
                if (robotStatus.getVerticalSide() == RobotStatus.VerticalSide.TOP) {
                        target = AllianceFlipUtil.apply(siteConstants.topLeftCenterPoint.toTranslation2d());
                } else {
                        target = AllianceFlipUtil.apply(siteConstants.topRightCenterPoint.toTranslation2d());
                }
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

                Translation2d vectorToTarget = target.minus(lookaheadPose.getTranslation());
                Rotation2d targetFieldAngle = vectorToTarget.getAngle();

                Pose2d simturretPosition = estimatedPose.transformBy(
                                new Transform2d(
                                                robotToTurret.getTranslation().toTranslation2d(),
                                                robotToTurret.getRotation().toRotation2d()));

                Logger.recordOutput("ToAlliancesimturretPosition",
                                new Pose2d(simturretPosition.getX(), simturretPosition.getY(), targetFieldAngle));
                return new ShootingState(targetFieldAngle, Hood_MAX_RADS, rollMap.get(lookaheadTurretToTargetDistance));
        }

}