package frc.robot.util.FMS;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotEvent.Event.*;

public class Signal extends SubsystemBase { // 類別名稱習慣大寫開頭
    public String gameData;
    public boolean CanGetPoint;
    private final List<TargetInactive> TargetInactive = new ArrayList<>();
    private final List<Targetactive> Targetactive = new ArrayList<>();
    private final double MatchTime = 140.0;
    private final double TRANSITION = 10.0;
    private final double Round = 25.0;
    private final double EndGame = 30.0;

    public Signal() {
    }

    public char getInactive() {
        gameData = DriverStation.getGameSpecificMessage();

        char allience = 'N';

        if (gameData != null && gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    allience = 'B';
                    break;
                case 'R':
                    allience = 'R';
                    break;
                default:
                    allience = 'N';
                    break;
            }
        }
        return allience;
    }
    

    public void TargetInactive(TargetInactive event) {
        TargetInactive.add(event);
    }

    public void Targetactive(Targetactive event) {
        Targetactive.add(event);
    }

    public void CanGetPointAllience() {
        if (DriverStation.isAutonomous()) {
            return;
        }
        if (DriverStation.getMatchTime() >= MatchTime - TRANSITION) {
            for(Targetactive listener : Targetactive){
                listener.Targetactive();
        }else if(DriverStation.getMatchTime() >= MatchTime - TRANSITION - Round){

        }
        }
    }

    @Override
    public void periodic() {

    }
}