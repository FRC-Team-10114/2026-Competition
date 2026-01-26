package frc.robot.subsystems.Intake.Roller;

import edu.wpi.first.units.measure.AngularVelocity;

public interface RollerIO {

    public void setVelocity(AngularVelocity velocity);

    public AngularVelocity getVelocity();

    public void configure();
}
