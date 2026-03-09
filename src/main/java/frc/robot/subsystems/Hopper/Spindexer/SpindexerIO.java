package frc.robot.subsystems.Hopper.Spindexer;

public interface SpindexerIO {
    
    public void run(double vlot);

    public void stop();

    public void configure();

    public double getStatorCurrent();
}
