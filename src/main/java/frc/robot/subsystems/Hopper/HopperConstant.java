package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.Amp;

import edu.wpi.first.units.measure.Current;

public class HopperConstant {

    public static final class SpindexerConstants {
        public static final Current MECHINE_STATOR_CURRENT_LIMIT = Amp.of(40);
        public static final Current MECHINE_SUPPLY_CURRENT_LIMIT = Amp.of(15);
    }
}
