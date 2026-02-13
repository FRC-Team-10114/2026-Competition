package frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Hopper.Spindexer.SpindexerIO;
import frc.robot.subsystems.Hopper.Spindexer.SpindexerIOHardware;
import frc.robot.subsystems.Shooter.Trigger.TriggerIO;
import frc.robot.subsystems.Shooter.Trigger.TriggerIOHardware;
import frc.robot.subsystems.Shooter.Trigger.TriggereNEO;

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

    public void warmUp() {
        this.spindexer.run(-2);
    }

    public void stopSpin() {
        this.spindexer.stop();
    }

    public void warmUpforshoot() {
        this.spindexer.run(-7);
    }

    public void stopAll() {
        this.stopSpin();
    }

    @Override
    public void periodic() {
    }
}
