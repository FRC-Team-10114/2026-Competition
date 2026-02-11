package frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Hopper.Spindexer.SpindexerIO;
import frc.robot.subsystems.Hopper.Spindexer.SpindexerIOHardware;
import frc.robot.subsystems.Hopper.Trigger.TriggerIO;
import frc.robot.subsystems.Hopper.Trigger.TriggerIOHardware;
import frc.robot.subsystems.Hopper.Trigger.TriggereNEO;

public class HopperSubsystem extends SubsystemBase {

    private final TriggerIO trigger;
    private final SpindexerIO spindexer;

    public HopperSubsystem(
            TriggerIO trigger,
            SpindexerIO spindexer) {
        this.trigger = trigger;
        this.spindexer = spindexer;
    }

    public static HopperSubsystem create() {
        return new HopperSubsystem(
                new TriggerIOHardware(),
                new SpindexerIOHardware());
    }

    public void warmUp() {
        this.spindexer.run(-7);
    }

    public void stopSpin() {
        this.spindexer.stop();
    }

    public void warmUpforshoot() {
        this.spindexer.run(-10);
    }

    public void load() {
        this.trigger.run();
    }

    public void stopTrigger() {
        this.trigger.stop();
    }

    public void stopAll() {
        this.stopTrigger();
        this.stopSpin();
    }

    @Override
    public void periodic() {
    }
}
