package frc.robot.subsystems.Shooter.Turret;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;

public interface TurretIO {
    public void setControl(Supplier<Angle> angleSupplier);
}
