package frc.robot.subsystems.Hopper;

import frc.robot.subsystems.Hopper.Spindexer.SpindexerIO;
import frc.robot.subsystems.Hopper.Trigger.TriggerIO;

public class HopperSubsystem {
    
    private final TriggerIO trigger;
    private final SpindexerIO spindexer;

    public HopperSubsystem(
        TriggerIO trigger,
        SpindexerIO spindexer
    ) {
        this.trigger = trigger;
        this.spindexer = spindexer;
    }

    public void warmUp() {
        this.spindexer.run();
    }

    public void load() {
        this.trigger.run();
    }

    public void stopTrigger() {
        this.trigger.stop();
    }

    public void stopSpin() {
        this.spindexer.stop();
    }
}
