package frc.robot.subsystems.Shooter;

import java.util.function.Supplier;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.Hood.HoodIO;
import frc.robot.subsystems.Shooter.Hood.HoodIO.HoodIOInputs;
import frc.robot.subsystems.Shooter.Roller.RollerIO;
import frc.robot.subsystems.Shooter.Turret.TurretIO;

public class ShooterSubsystem extends SubsystemBase {
    
    private final HoodIO hood;
    private final RollerIO roller;
    private final TurretIO turret;
    private final HoodIOInputs hoodIOInputs;

    public ShooterSubsystem(HoodIO hood, RollerIO roller, TurretIO turret) {

        this.hood = hood;
        this.roller = roller;
        this.turret = turret;
        this.hoodIOInputs = new HoodIOInputs();
    }


    @Override
    public void periodic() {
        hood.updateInputs(hoodIOInputs);
    }

    public void setHoodAngle(double targetRad) {
        this.hood.setAngle(targetRad); 
    }

    public void setTurretAngle(Supplier<Angle> targetRad) {
        this.turret.setControl(targetRad);
    }

    public void setRollerVelocity(AngularVelocity velocity) {
        this.roller.setRPM(velocity);
    }
}
