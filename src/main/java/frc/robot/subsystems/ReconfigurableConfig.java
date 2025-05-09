package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import frc.robot.shared.Limelight;
import frc.robot.subsystems.body.BodyConstants;
import frc.robot.subsystems.body.Elevator;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.swerve.TunerConstants;

public interface ReconfigurableConfig {
    public abstract void reconfigure()  ;

    public static List<Class<?>> RECONFIGS = new ArrayList<>();

    public static void addReconfigs() {
        if(RECONFIGS.isEmpty()){
            RECONFIGS.add(BodyConstants.class);
            RECONFIGS.add(ManipulatorConstants.class);
            RECONFIGS.add(TunerConstants.class);
            RECONFIGS.add(Limelight.class);
            RECONFIGS.add(Climber.class);
            RECONFIGS.add(Elevator.class);
        }
    }
    
    public static List<Class<?>> findReconfigurableConfigs() {
        addReconfigs() ;
        return RECONFIGS;
    }

    public static boolean isContainReconfig(String className) {
        addReconfigs() ;
        for (Class<?> reconfig : RECONFIGS) {
            if(reconfig.getName().equals(className)){
                return true ;
            }
        }
        return false ;

    }


   
}
