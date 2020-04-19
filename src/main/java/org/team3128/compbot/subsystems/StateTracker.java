package org.team3128.compbot.subsystems;

import org.team3128.common.generics.Threaded;
import org.team3128.common.utility.Log;
import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.Shooter.ShooterState;


public class StateTracker{
    public enum RobotState {
        SHORT_RANGE(Arm.ArmState.SHORT_RANGE, Shooter.ShooterState.SHORT_RANGE, "Short"),
        MID_RANGE(Arm.ArmState.MID_RANGE, Shooter.ShooterState.MID_RANGE, "Mid"),
        LONG_RANGE(Arm.ArmState.LONG_RANGE, Shooter.ShooterState.LONG_RANGE, "Long");

        public ArmState targetArmState;
        public ShooterState targetShooterState;
        public String shooterStateName;

        private RobotState(final ArmState armState, final ShooterState shooterState, String shooterStateName) {
            this.targetArmState = armState;
            this.targetShooterState = shooterState;
            this.shooterStateName = shooterStateName;
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
        robotState = RobotState.MID_RANGE;
    }

    public void setState(final RobotState desiredState) {
        robotState = desiredState;
    }

    public RobotState getState() {
        return robotState;
    }
}