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
import frc.robot.util.RobotStatus.RobotStatus;

public class PhotonVision extends SubsystemBase {

    private final PhotonCamera camera;
    private final CameraConfig config;
    private final PhotonPoseEstimator poseEstimator;
    private final CommandSwerveDrivetrain drivetrain;
    private boolean NeedResetPose = false;

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
    public void NeedResetPoseEvent() {
        this.NeedResetPose = true;
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
            if (this.NeedResetPose) {
                // 1. 發現旗標是 True！代表剛落地，需要強力校正
                // 給予極小的標準差 (例如 1公分, 1公分, 1度)
                stdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1));

                // 2. 【重要】用完之後馬上把旗標降下來 (False)
                // 這樣下一幀就會變回普通的信任度，不會一直鎖死
                this.NeedResetPose = false;

            } else {
                // 一般情況的標準差計算 (保持你原本的邏輯)
                if (numTags >= 2) {
                    stdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
                } else {
                    double distErr = 0.5 * Math.pow(avgDist, 2);
                    stdDevs = VecBuilder.fill(distErr, distErr, 999999);
                }
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
        // 1. 取得最新的一幀結果
        var result = camera.getLatestResult();

        // 2. 讓 PoseEstimator 嘗試計算座標
        // update() 會自動根據相機位置與 Tag 佈局算出 Field-Relative Pose
        var poseOpt = poseEstimator.update(result);

        if (poseOpt.isPresent()) {
            Pose3d estimatedPose3d = poseOpt.get().estimatedPose;
            Pose2d estimatedPose2d = estimatedPose3d.toPose2d();

            // 取得這一次計算用了幾顆 Tag
            int numTags = poseOpt.get().targetsUsed.size();

            // =========================================================
            // 安全過濾 (Safety Checks) - 寧可不重置，也不要重置錯
            // =========================================================

            // Check A: Z 軸高度檢查 (防飛天遁地)
            // 機器人應該在地面 (Z=0)。如果算出 Z > 0.5m，代表解算錯誤 (通常是誤判天花板燈光)
            if (Math.abs(estimatedPose3d.getZ()) > 0.5) {
                return false;
            }

            // Check B: 模糊度檢查 (防鏡像翻轉) - 僅針對單 Tag
            // 如果只看到 1 顆 Tag，數學上容易出現 "鏡像翻轉" 的解
            if (numTags == 1) {
                // 取得最佳目標的模糊度 (0.0~1.0)
                double ambiguity = result.getBestTarget().getPoseAmbiguity();

                // 如果模糊度 > 0.2，代表相機不確定角度，這時候重置風險太高
                if (ambiguity > 0.2) {
                    return false;
                }
            }
            // 註：如果 numTags >= 2，因為幾何三角定位的關係，幾乎不可能翻轉，所以不用檢查 Ambiguity

            // =========================================================
            // 3. 執行重置 (Execution)
            // =========================================================

            // 直接覆蓋里程計，視為絕對真理
            drivetrain.resetPose(estimatedPose2d);

            // (選用) Log 紀錄，方便比賽時確認有沒有成功 Reset
            // System.out.println("Vision Reset Success! Tags used: " + numTags);

            return true;
        }

        // 沒看到 Tag 或解算失敗
        return false;
    }

    // ** 外部 getter */
    public int getAprilTagId() {
        return m_lastTagId;
    }
}