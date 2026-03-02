package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ClimberConstants {
    
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);

    public static final double GEAR_RATIO = 1.0;
    public static final double POSITION_CONVERSION_FACTOR = GEAR_RATIO;

    public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(10.0);
    public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(20.0);

    public static final Angle FORWARD_LIMIT = Rotations.of(85.0);
    public static final Angle REVERSE_LIMIT = Rotations.of(-1.0);
}



