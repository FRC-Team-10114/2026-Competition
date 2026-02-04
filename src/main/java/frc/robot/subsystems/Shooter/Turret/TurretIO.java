package frc.robot.subsystems.Shooter.Turret;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.Shooter.Turret.TurretHardware.ShootState;

public interface TurretIO {

    public void setAngle(Rotation2d robotHeading, Angle targetRad, ShootState state);

    public void resetAngle();
}
