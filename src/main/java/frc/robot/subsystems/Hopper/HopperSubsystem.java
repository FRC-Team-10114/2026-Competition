package frc.robot.subsystems.Hopper;

import java.security.PublicKey;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Hopper.Spindexer.SpindexerIO;
import frc.robot.subsystems.Hopper.Spindexer.SpindexerIOHardware;

public class HopperSubsystem extends SubsystemBase {

    private final SpindexerIO spindexer;

    public HopperSubsystem(
            SpindexerIO spindexer) {
        this.spindexer = spindexer;
    }

    public static HopperSubsystem create() {
        return new HopperSubsystem(
                new SpindexerIOHardware());
    }

    public void stopSpin() {
        this.spindexer.stop();
    }

    public void warmUpforshoot() {
        this.spindexer.run(-5.0);
    }

    public void stopAll() {
        this.spindexer.run(0);
    }

    public void waiting() {
        this.spindexer.run(-1);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("spindexer", spindexer.getStatorCurrent());
    }
}
