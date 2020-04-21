package org.team3128.compbot.subsystems;

import org.team3128.common.utility.units.Length;

public class Fourbar {
    public enum FourbarState {
        GROUND(0 * Length.ft),
        MIDDLE(1 * Length.ft);
    
        public double targetHeight;

        private FourbarState(double height)
        {
            this.targetHeight = height;
        }
    }

    public enum ForkliftControlMode
	{
		PERCENT(2, "Percent Output"),
		POSITION_UP(0, "Position (Up)"),
		POSITION_DOWN(1, "Position (Down)");

		private int pidSlot;
		private String name;

		private ForkliftControlMode(int pidSlot, String name)
		{
			this.pidSlot = pidSlot;
			this.name = name;
		}

		public int getPIDSlot()
		{
			return pidSlot;
		}

		public String getName()
		{
			return name;
		}
    }
    
}