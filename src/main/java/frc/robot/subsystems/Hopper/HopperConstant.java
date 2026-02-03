package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.Amp;

import edu.wpi.first.units.measure.Current;

public class HopperConstant {
    
    public static final class TriggerConstants {
        public static final int TRIGGER_MOTOR_ID = 30;

        public static final Current TRIGGER_STATOR_CURRENT_LIMIT = Amp.of(60);
        public static final Current TRIGGER_SUPPLY_CURRENT_LIMIT = Amp.of(50);
    }

    public static final class SpindexerConstants {
        public static final int MECHINE_MOTOR_ID = 31;

        public static final Current MECHINE_STATOR_CURRENT_LIMIT = Amp.of(60);
        public static final Current MECHINE_SUPPLY_CURRENT_LIMIT = Amp.of(50);
    }
}
