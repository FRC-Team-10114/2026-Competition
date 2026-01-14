package frc.robot.subsystems.Shooter.Hood;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class HoodIONEO implements HoodIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    public HoodIONEO(int canID) {
            motor = new SparkMax(canID, MotorType.kBrushless);

            encoder = motor.getEncoder();
            pidController = motor.getClosedLoopController();

            motor.configure(HoodConfig.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
        
    @Override
    public void setAngle(double rad) {
        pidController.setSetpoint(rad, ControlType.kPosition);
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
