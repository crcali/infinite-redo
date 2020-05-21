package org.team3128.compbot.subsystems;

import org.team3128.common.generics.Threaded;

public class Arm extends Threaded {

    public double ratio, error;

    public static enum ArmState {

        VERTICAL(90 * Angle.DEGREES),
        DOWN(45 * Angle.DEGREES);

        public double targetAngle;

        private ArmState(double targetAngle) {
            this.targetAngle = targetAngle;
		}

    }

    private enum ArnControlMode {
        PERCENT(1, "Percent Output"),
		POSITION(1, "Position"),
        ZEROING(1, "Zeroing");
        
        private int pidSlot;
        private String name;

        private ArmControlMode(int pidSlot, String name) {
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

    public void setControlMode(FourbarControlMode mode) {
        if (mode != controlMode) {
            controlMode = mode;
            Log.debug("FourBar", "Setting control mode to " + mode.name() + ", with PID slot" + mode.getPIDSlot);
            fourBarMotor.selectProfileSlot(mode.getPIDSlot(), 0);
        }
    }
    public void setState(ArmState armState) {
        state = armState;

        Log.info("Arm", "Going to " + state.targetAngle + "degrees");
        angleControl(state.targetAngle)
    }

    public LazyCANSparkMax fourBarMotor;
    FourBarControlMode controlMode;
    FourBarState state;

    public DigitalInput limitSwitch;
    double limitSwitchAngle;
    int maxVelocity;

    public boolean override = false;
    
    public double maxAngle = +95.0 * Angle.DEGREES; // TODO: Add to constants class
	public double minAngle = -90.0 * Angle.DEGREES;

	public boolean disabled = false;

	public boolean canLower = false;
	public boolean canRaise = true;

	private double desiredTarget = 0;

	private double lastTime;
	private double lastError;

	private double joystickThreshold = 0.1;

    private static Arm instance = null;
	public static Arm getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal("Arm", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
    }
    
    public static void initialize(LazyCANSparkMax armMotor, ArmState state, DigitalInput limitSwitch, double ratio, double limitSwitchAngle, int maxVelocity) {
		instance = new FourBar(armMotor, state, limitSwitch, ratio, limitSwitchAngle, maxVelocity);
    }

    private void configMotors() {
        armMotor = new LazyTalonFX(1);
    }
    
    private void configSensors() {
        limitSwitch = new DigitalInput(1);
    }

    private Arm(LazyCANSparkMax armMotor, ArmState state, DigitalInput limitSwitch, double ratio, double limitSwitchAngle, int maxVelocity) {
        configMotors();
        configSensors();
        setState
    }
    
    public void angleControl(double angle) {
		setControlMode(ArmControlMode.POSITION);

		desiredTarget = angle;

		lastTime = RobotController.getFPGATime();
		lastError = desiredTarget - this.getCurrentAngle();
    }
    
    public boolean getLimitSwitch()
	{
		return !limitSwitch.get();
    }
    
    public void brake() {
		angleControl(this.getCurrentAngle());
    }
    
    public void zero() {
		setControlMode(ArmControlMode.ZEROING);

		desiredTarget = ZEROING_VELOCITY;

		lastTime = RobotController.getFPGATime();
		lastError = desiredTarget - fourBarMotor.getSelectedSensorVelocity();
	}

    @Override
    public void update() {   
        // int zeroVelocityCount = 0;

		// 	double currentTarget = 0;
		// 	double previousTarget = 0;

		// 	double kP;
        //     double kD;
            
        //     if (this.controlMode == ArmControlMode.ZEROING) {
        //         if (Math.abs(armMotor.getSelectedSensorVelocity()) < 2) {
        //             zeroVelocityCount += 1;
        //         }
        //         else {
        //             zeroVelocityCount = 0;
        //         }

        //         if (zeroVelocityCount > 30 || this.getLimitSwitch()) {
        //             this.state = FourBarState.VERTICAL;
        //             Log.info("Arm", "Zeroing sequence hit hard/soft stop. Braking now...");

        //             this.angleControl(85.0 * Angle.DEGREES);

        //             zeroVelocityCount = 0;
        //         }
        //     }

        //     else if (this.getLimitSwitch()) {
        //         this.setCurrentAngle(this.limitSwitchAngle);

        //         this.state = ArmState.VERTICAL;
        //     }

        //     else if (this.disabled) {
        //         this.armMotor.set(ControlMode.PercentOutput, 0);
        //     }

        //     else {
        //         currentTarget = 0;

        //         if (this.controlMode == ArmControlMode.PERCENT) {
        //             if (this.override) {
        //                 currentTarget = desiredTarget;
        //                 this.armMotor.set(ControlMode.PercentOutput, currentTarget);
        //             }
        //             else {
        //                 this.canRaise = this.getCurrentAngle() < this.maxAngle - 2 * Angle.DEGREES;
        //                 this.canLower = this.getCurrentAngle() > this.minAngle + 1 * Angle.DEGREES;

        //                 if (desiredTarget > 0 && this.canRaise) {
        //                     currentTarget = 0.7 * getAdjustedTarget(desiredTarget);
        //                 }
        //                 else if (desiredTarget < 0 && this.canLower) {
        //                     currentTarget = 0.4 * getAdjustedTarget(desiredTarget);
        //                 }

        //                 if ((Math.abs(currentTarget) < 0.0001 && this.canRaise && this.canLower)) {
        //                     this.brake();
        //                 }
        //             }
        //         }
        //         else if (this.controlMode == ArmControlMode.POSITION) {
        //             lastError = this.error;
        //             this.error = desiredTarget - this.getCurrentAngle();

        //             if (this.error > 0) {
        //                 kP = 0.23;
        //             }
        //             else {
        //                 kP = 0.01;
        //             }
        //             kD = 0;

        //             currentTarget = this.getFeedForwardPower() + kP * this.error + kD * (this.error - lastError) * 1000000 / (RobotController.getFPGATime() - this.lastTime);
        //             this.lastTime = RobotController.getFPGATime();
        //         }
        //         else if (this.controlMode == ArmControlMode.ZEROING) {
        //             lastError = this.error;
        //             this.error = desiredTarget - fourBarMotor.getSelectedSensorVelocity();

        //             currentTarget = 0.8272;
        //             this.lastTime = RobotController.getFPGATime();
        //         }

        //         if (Math.abs(currentTarget - previousTarget) > 0.0001) {
        //             this.armBarMotor.set(ControlMode.PercentOutput, currentTarget);

        //             previousTarget = currentTarget;
        //         }
        //     }

        //     try
        //     {
        //         Thread.sleep(10);
        //     }
        //     catch (InterruptedException e)
        //     {
        //         e.printStackTrace();
        //     }
        // }
        
    }
}