package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.Constants.PhotonVisionConstants.CameraConfig;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class PhotonVision extends SubsystemBase {

    private final PhotonCamera camera;
    private final CameraConfig config;
    private final PhotonPoseEstimator poseEstimator;
    private final CommandSwerveDrivetrain drivetrain;

    private int m_lastTagId = -1;

    public PhotonVision(CommandSwerveDrivetrain drive, CameraConfig config) {
        this.drivetrain = drive;
        this.config = config;

        // 1. 初始化相機（名稱從 Constants 取得）
        this.camera = new PhotonCamera(config.cameraName());

        // 2. 初始化場地與 Estimator
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, config.cameraLocation());

        this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        updateVision();
    }

    private void updateVision() {
        // 讀取所有未讀取的結果（對應高頻率相機）
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

            var poseOpt = poseEstimator.update(result);
            if (poseOpt.isEmpty())
                continue;

            // 基礎資訊更新
            if (result.hasTargets()) {
                m_lastTagId = result.getBestTarget().getFiducialId();
            }

            // 過濾 A：旋轉速度過快（防止動態模糊導致的誤判）
            double rotSpeed = Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
            if (rotSpeed > PhotonVisionConstants.maxYawRate)
                continue;

            var est = poseOpt.get();
            Pose3d estimatedPose3d = est.estimatedPose;

            // 過濾 B：Z 軸高度檢查（機器人不可能飛起來或鑽進地裡）
            if (Math.abs(estimatedPose3d.getZ()) > 0.5)
                continue;

            // 過濾 C：單個 Tag 的有效性檢查
            int numTags = est.targetsUsed.size();
            double avgDist = 0.0;
            for (var tgt : est.targetsUsed) {
                avgDist += tgt.getBestCameraToTarget().getTranslation().getNorm();
            }
            avgDist /= numTags;

            if (numTags == 1) {
                // 單 Tag 若太遠或太模糊則捨棄
                if (avgDist > PhotonVisionConstants.maxSingleTagDistanceMeters)
                    continue;
                if (result.getBestTarget().getPoseAmbiguity() > 0.2)
                    continue;
            }

            // 3. 計算信任權重（標準差）
            Vector<N3> stdDevs;
            if (numTags >= 2) {
                // 多 Tag 極其信任
                stdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
            } else {
                // 單 Tag 信任度隨距離平方衰減，且完全不信任視覺的角度（交給陀螺儀）
                double distErr = 0.5 * Math.pow(avgDist, 2);
                stdDevs = VecBuilder.fill(distErr, distErr, 999999);
            }

            // 4. 發送至 Drivetrain
            double timestamp = Utils.fpgaToCurrentTime(est.timestampSeconds);
            drivetrain.addVisionMeasurement(estimatedPose3d.toPose2d(), timestamp, stdDevs);

            Logger.recordOutput("Vision/PhotonVision/" + config.cameraName() + "/Pose", estimatedPose3d);
            Logger.recordOutput("Vision/PhotonVision/" + config.cameraName() + "/timestamp", timestamp);
            Logger.recordOutput("Vision/PhotonVision/" + config.cameraName() + "/stdDevs", stdDevs);
        }
    }

    /** 用於自動階段對齊或初始定位 */
    public boolean resetPoseToVision() {
        var result = camera.getLatestResult();
        var poseOpt = poseEstimator.update(result);

        if (poseOpt.isPresent()) {
            Pose2d estimatedPose2d = poseOpt.get().estimatedPose.toPose2d();
            Pose3d estimatedPose3d = poseOpt.get().estimatedPose;

            // 1. 基礎檢查：高度是否合理 (Z 軸)
            if (Math.abs(estimatedPose3d.getZ()) > 0.5)
                return false;

            // 2. 取得目前機器人的位置
            Pose2d currentPose = drivetrain.getPose2d();

            // 3. 計算視覺與目前位置的差異
            // 距離差異 (公尺)
            double distanceDiff = currentPose.getTranslation().getDistance(estimatedPose2d.getTranslation());
            // 角度差異 (度)
            double rotationDiff = Math
                    .abs(currentPose.getRotation().getDegrees() - estimatedPose2d.getRotation().getDegrees());

            // 4. 設定門檻 (Thresholds)
            // 如果差異太小 (例如距離 < 5cm 且 角度 < 2度)，就視為已經精準，不需要重設
            double distanceTolerance = 0.05; // 5 公分
            double rotationTolerance = 2.0; // 2 度

            if (distanceDiff < distanceTolerance && rotationDiff < rotationTolerance) {
                // 差異太小，不執行重設，回傳 true 或 false 視你的邏輯而定 (這裡回傳 false 代表沒重設)
                return false;
            }

            // 5. 只有差異夠大才重設座標
            drivetrain.resetPose(estimatedPose2d);
            return true;
        }

        return false;
    }

    // ** 外部 getter */
    public int getAprilTagId() {
        return m_lastTagId;
    }
}