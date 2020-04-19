/*package org.team3128.compbot.subsystems;

import org.team3128.common.generics.Threaded;
import org.team3128.common.utility.Log;


public class StateTracker{
    public enum RobotState {
        
        private RobotState() {
            
        }
        
    }
    
    private static StateTracker instance = new StateTracker();
    public static RobotState robotState;

    public static StateTracker getInstance(){
        if (instance != null) {
            return instance;
        }

        Log.fatal("StateTracker", "Attempted to get instance before initialization! Call initialize(...) first.");
        return null;
    }

    private StateTracker(){

    }

    public void setState(final RobotState desiredState) {
        robotState = desiredState;
    }

    public RobotState getState() {
        return robotState;
    }
}*/