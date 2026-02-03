package frc.robot.util.RobotEvent;

public class Event {
    public Event() {
    }

    @FunctionalInterface
    public interface NeedResetPoseEvent {
        void NeedResetPose();
    }

    @FunctionalInterface
    public interface TargetInactive{
        void TargetInactive();
    }
    @FunctionalInterface
    public interface Targetactive{
        void Targetactive();
    }
}