package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ClimberConstants {
    
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(50);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);

    public static final double GEAR_RATIO = 1.0;
    public static final double ROTATION_PER_METER = 1.0;
    public static final double POSITION_CONVERSION_FACTOR = GEAR_RATIO * ROTATION_PER_METER;

    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(2);
    public static final LinearVelocity CRUISE_VELOCITY = MetersPerSecond.of(1);

    public static final Distance FORWARD_LIMIT = Meters.of(5.0);
    public static final Distance REVERSE_LIMIT = Meters.of(0.05);
}



