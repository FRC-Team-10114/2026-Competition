package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IDs;

public class ClimberIOSpark implements ClimberIO {

    private final SparkFlex master;
    private final SparkFlex slave;
    private final RelativeEncoder climberEncoder;
    private final SparkClosedLoopController climberController;
    private final SparkClosedLoopController climberController2;

    public ClimberIOSpark() {
        this.master = new SparkFlex(IDs.Climber.CLIMBER_MOTOR, MotorType.kBrushless);
        this.slave = new SparkFlex(IDs.Climber.CLIMBER_SLAVE_MOTOR, MotorType.kBrushless);

        this.climberEncoder = master.getEncoder();
        this.climberController = master.getClosedLoopController();
        this.climberController2 = slave.getClosedLoopController();

        configure();
        resetPosition();
    }

    @Override
    public void setPosition(double round) {
        this.climberController.setSetpoint(round,
                ControlType.kPosition);
        this.climberController2.setSetpoint(round,
                ControlType.kPosition);
    }

    @Override
    public double getPosition() {
        return this.climberEncoder.getPosition();
    }

    @Override
    public void setVolt(double voltage) {
        this.master.setVoltage(voltage);
        this.slave.setVoltage(voltage);
    }

    @Override
    public void resetPosition() {
        this.climberEncoder.setPosition(0.0);
    }
    @Override
    public double maingetOutputCurrent() {
        // 讀取 SparkFlex 的實際輸出負載電流 (安培)
        return this.master.getOutputCurrent();
    }
    @Override
    public double getOutputCurrent() {
        return this.slave.getOutputCurrent();
    }
    

    @SuppressWarnings("removal")
    @Override
    public void configure() {
        var masterConfig = new SparkFlexConfig();
        var slaveConfig = new SparkFlexConfig();

        // 1. 設定 Master (純淨的鏈式呼叫)
        masterConfig
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit((int) ClimberConstants.STATOR_CURRENT_LIMIT.in(Amps));

        masterConfig.encoder
                .positionConversionFactor(ClimberConstants.POSITION_CONVERSION_FACTOR);

        // 🟢 將 PID 掛載到 Master 身上
        masterConfig.closedLoop
                .pid(1.0, 0.0, 0.0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        // 🟢 將 RPS 轉換為 REV MAXMotion 底層強制要求的 RPM 單位
        double cruiseRPM = ClimberConstants.CRUISE_VELOCITY.in(RotationsPerSecond) * 60.0;
        double accelRPM = ClimberConstants.MAX_ACCELERATION.in(RotationsPerSecondPerSecond) * 60.0;

        masterConfig.closedLoop.maxMotion
                .maxAcceleration(accelRPM)
                .cruiseVelocity(cruiseRPM)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        // 🟢 軟體限位 (使用 .in(Rotations) 確保單位無誤)
        masterConfig.softLimit
                .forwardSoftLimit(ClimberConstants.FORWARD_LIMIT.in(Rotations))
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimit(ClimberConstants.REVERSE_LIMIT.in(Rotations))
                .reverseSoftLimitEnabled(false);

        // 1. 設定 Master (純淨的鏈式呼叫)
        slaveConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit((int) ClimberConstants.STATOR_CURRENT_LIMIT.in(Amps));

        slaveConfig.encoder
                .positionConversionFactor(ClimberConstants.POSITION_CONVERSION_FACTOR);

        // 🟢 將 PID 掛載到 Master 身上
        slaveConfig.closedLoop
                .pid(1.0, 0.0, 0.0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        slaveConfig.closedLoop.maxMotion
                .maxAcceleration(accelRPM)
                .cruiseVelocity(cruiseRPM)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        // 🟢 軟體限位 (使用 .in(Rotations) 確保單位無誤)
        slaveConfig.softLimit
                .forwardSoftLimit(ClimberConstants.FORWARD_LIMIT.in(Rotations))
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimit(ClimberConstants.REVERSE_LIMIT.in(Rotations))
                .reverseSoftLimitEnabled(false);

        // 3. 一次性安全寫入馬達配置// 🟢 將 ResetMode 改成官方最新推薦的 kNoResetSafeParameters
        this.master.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.slave.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
