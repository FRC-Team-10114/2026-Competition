package frc.robot.subsystems.Shooter.Trigger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class TriggerIOSpark implements TriggerIO {
    private final SparkFlex TriggertMotor;
    private final SparkClosedLoopController TriggerController;
    private final RelativeEncoder TriggerEncoder;

    public TriggerIOSpark() {
        this.TriggertMotor = new SparkFlex(50, MotorType.kBrushless);
        this.TriggerController = TriggertMotor.getClosedLoopController();
        this.TriggerEncoder = TriggertMotor.getEncoder();
    }

    public void run() {
        this.TriggertMotor.set(1);
    }

    public void stop() {
        this.TriggertMotor.stopMotor();
    }

    public void configure() {
        var TriggerConfig = new SparkFlexConfig();

        TriggerConfig
                .smartCurrentLimit(40) // 建議 40A 就夠了
                .idleMode(IdleMode.kBrake)
                .inverted(false);

        this.TriggertMotor.configure(
                TriggerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }
}
