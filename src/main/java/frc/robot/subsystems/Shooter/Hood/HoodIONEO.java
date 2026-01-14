package frc.robot.subsystems.Shooter.Hood;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class HoodIONEO implements HoodIO {
    
    public class HoodIOSparkMax implements HoodIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    private final double GEAR_RATIO = 50.0; // 假設減速比 50:1
    private final double POSITION_CONVERSION_FACTOR = (2 * Math.PI) / GEAR_RATIO;

    public HoodIOSparkMax(int canID) {
        motor = new SparkMax(canID, MotorType.kBrushless);

        encoder = motor.getEncoder();
        pidController = motor.getClosedLoopController();

        // 設定轉換因子，讓 encoder.getPosition() 直接回傳弧度(Radians)
        encoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);

        // 設定 PID 參數
        pidController.setP(0.5); // 需根據實機調整
        pidController.setI(0.0);
        pidController.setD(0.0);
        pidController.setOutputRange(-0.5, 0.5); // 限制最大輸出，防止暴力撞牆

        // 設定軟極限 (Soft Limits) - 非常重要！
        motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Math.toRadians(60)); // 上限 60度
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) Math.toRadians(10)); // 下限 10度

        motor.burnFlash(); // 儲存設定至馬達內
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.positionRad = encoder.getPosition();
        inputs.velocityRadPerSec = encoder.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setAngle(double rad) {
        // 使用 SparkMax 內建的 Position PID 控制
        pidController.setReference(rad, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
}
