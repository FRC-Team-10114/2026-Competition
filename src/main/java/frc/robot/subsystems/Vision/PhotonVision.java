package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class PhotonVision extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private boolean NeedResetPose = false;
    private int m_lastTagId = -1;

    private static class CamWrapper {
        final String name;
        final PhotonCamera cam;
        final PhotonPoseEstimator estimator;

        CamWrapper(String name, PhotonCamera cam, PhotonPoseEstimator estimator) {
            this.name = name;
            this.cam = cam;
            this.estimator = estimator;
        }
    }

    private final double borderPixels = PhotonVisionConstants.borderPixels;
    private final double maxSingleTagDistanceMeters = PhotonVisionConstants.maxSingleTagDistanceMeters;

    private final List<CamWrapper> cams = new ArrayList<>();

    public PhotonVision(CommandSwerveDrivetrain drive, Map<String, Transform3d> cameraTransforms) {
        this.drivetrain = drive;
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        PhotonVisionConstants.cameraTransforms.forEach((name, transform) -> {
            PhotonCamera cam = new PhotonCamera(name);
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(
                    fieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    transform);
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            cams.add(new CamWrapper(name, cam, estimator));
        });
    }

    @Override
    public void periodic() {
        updateVision();
    }

    public void NeedResetPoseEvent() {
        this.NeedResetPose = true;
    }

    private void updateVision() {
        // 讀取所有相機
        for (CamWrapper cw : cams) {
            // 讀取未讀取的結果 (處理高幀率)
            for (PhotonPipelineResult result : cw.cam.getAllUnreadResults()) {
                
                // 基礎資訊更新
                if (result.hasTargets()) {
                    m_lastTagId = result.getBestTarget().getFiducialId();
                }

                Optional<EstimatedRobotPose> poseOpt = cw.estimator.update(result);
                if (poseOpt.isEmpty())
                    continue;

                EstimatedRobotPose est = poseOpt.get();
                Pose3d cameraRobotPose3d = est.estimatedPose;
                double resultTimeSec = est.timestampSeconds; // 這是 FPGA Timestamp

                // ---------------------------------------------------------
                // 過濾 A：旋轉速度過快（防止動態模糊導致的誤判）
                // ---------------------------------------------------------
                double rotSpeed = Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
                if (rotSpeed > PhotonVisionConstants.maxYawRate)
                    continue;

                // ---------------------------------------------------------
                // 過濾 B：Z 軸高度檢查
                // ---------------------------------------------------------
                if (!filterByZ(cameraRobotPose3d))
                    continue;

                // ---------------------------------------------------------
                // 過濾 C：邊緣檢測 (防止畸變)
                // ---------------------------------------------------------
                boolean cornerNearEdge = false;
                var targets = result.getTargets();
                // 獲取相機解析度 (假設 Constant 有定義，或是預設值)
                double resolutionX = 1280; // 範例值，請改為 cw.cam.getCameraMatrix() 獲取或 Constants
                double resolutionY = 720;  // 範例值

                // 嘗試動態獲取解析度，若失敗則忽略右下邊界檢查
                if(cw.cam.getCameraMatrix().isPresent()) {
                     resolutionX = cw.cam.getCameraMatrix().get().getNumCols();
                     resolutionY = cw.cam.getCameraMatrix().get().getNumRows();
                }

                for (var tgt : targets) {
                    var corners = tgt.getDetectedCorners();
                    if (corners != null) {
                        for (var corner : corners) {
                            if (corner.x < borderPixels || corner.y < borderPixels ||
                                corner.x > resolutionX - borderPixels ||
                                corner.y > resolutionY - borderPixels) {
                                cornerNearEdge = true;
                                break;
                            }
                        }
                    }
                    if (cornerNearEdge) break;
                }
                if (cornerNearEdge) continue;

                // ---------------------------------------------------------
                // 計算 Tag 數量與平均距離
                // ---------------------------------------------------------
                int numTags = est.targetsUsed.size();
                double avgDist = 0.0;
                for (var tgt : est.targetsUsed) {
                    avgDist += tgt.getBestCameraToTarget().getTranslation().getNorm();
                }
                if (numTags > 0) avgDist /= numTags;

                // ---------------------------------------------------------
                // 過濾 D：單 Tag 有效性檢查
                // ---------------------------------------------------------
                if (numTags < 2) {
                    // 如果只有一個 Tag，距離太遠則丟棄
                    if (avgDist > maxSingleTagDistanceMeters) continue;
                    
                    // 如果只有一個 Tag，Ambiguity 太高則丟棄
                    if (result.getBestTarget() != null && result.getBestTarget().getPoseAmbiguity() > 0.2) continue;
                }

                // ---------------------------------------------------------
                // 計算信任權重 (Standard Deviations)
                // ---------------------------------------------------------
                Vector<N3> stdDevs;
                if (this.NeedResetPose) {
                    // 強制重置模式：給予極高信任度
                    stdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(1));
                    this.NeedResetPose = false; // 重置旗標
                } else {
                    // 一般模式
                    if (numTags >= 2) {
                        // 多 Tag：信任度高
                        stdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
                    } else {
                        // 單 Tag：信任度隨距離下降
                        double distErr = 0.5 * Math.pow(avgDist, 2);
                        stdDevs = VecBuilder.fill(distErr, distErr, 100.0); // 旋轉信任度給低一點
                    }
                }

                // ---------------------------------------------------------
                // 發送至 Drivetrain
                // ---------------------------------------------------------
                drivetrain.addVisionMeasurement(
                    cameraRobotPose3d.toPose2d(), 
                    resultTimeSec, // 直接使用 PhotonVision 的 Timestamp
                    stdDevs
                );
            }
        }
    }

    /**
     * 用於自動階段初始化或特殊情況，尋找最可信的 Vision Pose 並強制覆蓋 Odometry
     */
    public boolean resetPoseToVision() {
        Pose2d bestPose = null;
        double minScore = 99999.0; // 分數越低越好
        
        for (CamWrapper cw : cams) {
            PhotonPipelineResult result = cw.cam.getLatestResult();
            if (!result.hasTargets()) continue;

            Optional<EstimatedRobotPose> poseOpt = cw.estimator.update(result);
            if (poseOpt.isEmpty()) continue;

            EstimatedRobotPose est = poseOpt.get();
            Pose3d pose3d = est.estimatedPose;

            // 1. 高度檢查
            if (Math.abs(pose3d.getZ()) > 0.5) continue;

            // 2. 計算 Tag 資訊
            int numTags = est.targetsUsed.size();
            double avgDist = 0.0;
            for (var tgt : est.targetsUsed) {
                avgDist += tgt.getBestCameraToTarget().getTranslation().getNorm();
            }
            if (numTags > 0) avgDist /= numTags;

            // 3. 過濾模糊單 Tag
            if (numTags == 1) {
                var bestTarget = result.getBestTarget();
                if (bestTarget != null && bestTarget.getPoseAmbiguity() > 0.2) continue;
            }

            // 4. 評分 (多 Tag 優先，近距離優先)
            double currentScore;
            if (numTags >= 2) {
                currentScore = avgDist; // 多 Tag 直接比距離
            } else {
                currentScore = 100.0 + avgDist; // 單 Tag 分數加權，讓它永遠輸給多 Tag
            }

            // 更新最佳結果
            if (currentScore < minScore) {
                minScore = currentScore;
                bestPose = pose3d.toPose2d();
            }
        }

        if (bestPose != null) {
            drivetrain.resetPose(bestPose);
            return true;
        }

        return false;
    }

    public int getAprilTagId() {
        return m_lastTagId;
    }

    private boolean filterByZ(Pose3d pose3d) {
        return Math.abs(pose3d.getZ()) < PhotonVisionConstants.maxZ;
    }
}