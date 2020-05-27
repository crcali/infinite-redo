package org.team3128.compbot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.team3128.compbot.subsystems.Constants;
import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyTalonFX;
import org.team3128.common.utility.units.Length;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.Log;


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

    public static enum ArmControlMode {

        PERCENT(1, "Percent Output"),
		POSITION(1, "Position"),
        ZEROING(1, "Zeroing");
        
        public int pidSlot;
        public String name;

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

    public LazyTalonFX armMotor;
    ArmControlMode controlMode;
    ArmState state;

    public DigitalInput limitSwitch;
    double limitSwitchAngle;
    int maxVelocity;

    public boolean override = false;

	public boolean disabled = false;

	public boolean canLower = false;
	public boolean canRaise = true;

	private double desiredTarget = 0;
    public double output = 0;
	private double lastTime;
	private double prevError;

	private double joystickThreshold = 0.1;

    private static Arm instance = null;
	public static Arm getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal("Arm", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
    }
    
    public static void initialize(LazyTalonFX armMotor, ArmState state, DigitalInput limitSwitch, double ratio, double limitSwitchAngle, int maxVelocity) {
		instance = new Arm(armMotor, state, limitSwitch, ratio, limitSwitchAngle, maxVelocity);
    }

    private void configMotors() {
        armMotor = new LazyTalonFX(1);
    }
    
    private void configSensors() {
        limitSwitch = new DigitalInput(1);
    }

    private Arm(LazyTalonFX armMotor, ArmState state, DigitalInput limitSwitch, double ratio, double limitSwitchAngle, int maxVelocity) {
        configMotors();
        configSensors();
        setState(state);
    }

    public void setControlMode(ArmControlMode mode) {
        if (mode != controlMode) {
            controlMode = mode;
            Log.debug("Arm", "Setting control mode to " + mode.name() + ", with PID slot" + mode.getPIDSlot());
            armMotor.selectProfileSlot(mode.getPIDSlot(), 0);
        }
    }
    
    public void setState(ArmState armState) {
        state = armState;
        Log.info("Arm", "Going to " + state.targetAngle + "degrees");
        angleControl(state.targetAngle);
    }
    
    public void angleControl(double angle) {
		setControlMode(ArmControlMode.POSITION);

		desiredTarget = angle;

		lastTime = RobotController.getFPGATime();
		prevError = desiredTarget - this.getAngle();
    }
    
    public boolean getLimitSwitch()
	{
		return !limitSwitch.get();
    }
    
    public void brake() {
		angleControl(this.getAngle());
    }
    
    public void zero() {
		setControlMode(ArmControlMode.ZEROING);

		desiredTarget = 0;

		lastTime = RobotController.getFPGATime();
		prevError = desiredTarget - armMotor.getSelectedSensorVelocity();
	}

    public double getAngle() {
        return (((getEncoderPos() / Constants.MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION)
                / Constants.ArmConstants.ARM_GEARING) * 360) % 360; 
    }

    public int getEncoderPos() {
        return armMotor.getSelectedSensorPosition(0);
    }
    
    @Override
    public void update() {   

        int plateauCount = 0;

        double currentTarget = getAngle();
        double error = desiredTarget - currentTarget;

        double kP_term = Constants.ArmConstants.ARM_PID.kP * error;
        double kD_term = Constants.ArmConstants.ARM_PID.kD * (error - prevError) / Constants.MechanismConstants.DT;

		double voltage_output = armFeedForward(desiredTarget) + kP_term + kD_term;
        double voltage = RobotController.getBatteryVoltage();

        output = voltage_output / voltage;
        if (output > 1) {
            // Log.info("ARM",
            //         "WARNING: Tried to set power above available voltage! Saturation limit SHOULD take care of this");
            output = 1;
        } else if (output < -1) {
            // Log.info("ARM",
            //         "WARNING: Tried to set power above available voltage! Saturation limit SHOULD take care of this ");
            output = -1;
        }

        if (Math.abs(error) < Constants.ArmConstants.ANGLE_THRESHOLD) {
            plateauCount++;
        } else {
            plateauCount = 0;
        }


        if((desiredTarget == 0) && !getLimitSwitch()) {
            output = Constants.ArmConstants.ZEROING_VELOCITY;
            // Log.info("Arm", "Using ZEROING_POWER to finish zeroing the arm.");
        } else if((desiredTarget == 0) && getLimitSwitch()) {
            output = 0;
            // Log.info("Arm", "In zero position, setting output to 0.");
        }


        armMotor.set(ControlMode.PercentOutput, output);

        prevError = error;
        
    }
    public double armFeedForward(double desired) {
        return 0;
    }
}