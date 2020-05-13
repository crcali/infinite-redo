package org.team3128.compbot.subsystems;

import org.team3128.common.generics.Threaded;
import org.team3128.compbot.subsystems.Constants;
import org.team3128.common.utility.units.Length;

public class Lift extends Threaded {
    public enum LiftState {
        DOWN(0 * Length.in),
        MIDDLE(Constants.LiftConstants.MIDDLE_DISTANCE),
        TOP(Constants.LiftConstants.TOP_DISTANCE);

        public double liftHeight;

        private LiftState(double liftHeight) {
            this.liftHeight = liftHeight;
        }
    }

    @Override
    public void update() {    

    }
}