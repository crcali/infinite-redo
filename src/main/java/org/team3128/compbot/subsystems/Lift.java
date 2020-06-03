package org.team3128.compbot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.team3128.common.generics.Threaded;
import org.team3128.compbot.subsystems.Constants;
import org.team3128.common.utility.units.Length;
import org.team3128.common.hardware.motor.LazyTalonFX;
import org.team3128.common.utility.Log;

public class Lift extends Threaded {
    public enum LiftState {
        ZERO(0 * Length.in),
        MIDDLE(Constants.LiftConstants.MIDDLE_DISTANCE),
        TOP(Constants.LiftConstants.TOP_DISTANCE);

        public double liftHeight;

        private LiftState(double liftHeight) {
            this.liftHeight = liftHeight;
        }
    }

	public double error;

    public LazyTalonFX liftMotor;
    public LiftState state; 
    DigitalInput limitSwitch;

    public boolean canLower = false;
	public boolean canRaise = true;

	private double desiredTarget = 0;
	private double setPoint = 0;

    public boolean override = false;

    private static Lift instance = null;

    public double output = 0;
	private double lastTime;
	private double prevError;
    
    public static Lift getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal("Lift", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
    }

    private void configMotors() {
        liftMotor = new LazyTalonFX(2);
    }
    
    private void configSensors() {
        limitSwitch = new DigitalInput(2);
    }

    public static void initialize(LiftState state, LazyTalonFX liftMotor, DigitalInput limitSwitch) {
		instance = new Lift(state, liftMotor, limitSwitch);
    }

	private Lift(LiftState state, LazyTalonFX liftMotor, DigitalInput limitSwitch) {
        configMotors();
        configSensors();
        setState(state);
    }

    public boolean getLimitSwitch()
	{
		return !limitSwitch.get();
    }

    public double getCurrentHeight() {
		return liftMotor.getSelectedSensorPosition(0) / Constants.LiftConstants.RATIO;
    }
    
    public void setState(LiftState state){
        state = state;
        Log.info("Lift", "Going to " + state.liftHeight + "inches");
        positionControl(state.liftHeight);
    }

    public void positionControl(double height) {
		desiredTarget = height;

		lastTime = RobotController.getFPGATime();
		prevError = desiredTarget - this.getCurrentHeight();
    }

    @Override
    public void update() {    
        
        int plateauCount = 0;

        double currentTarget = getCurrentHeight();
        double error = desiredTarget - currentTarget;

        double kP_term = Constants.LiftConstants.LIFT_PID.kP * error;
        double kD_term = Constants.LiftConstants.LIFT_PID.kD * (error - prevError) / Constants.MechanismConstants.DT;

		double voltage_output = liftFeedForward(desiredTarget) + kP_term + kD_term;
        double voltage = RobotController.getBatteryVoltage();

        output = voltage_output / voltage;
        if (output > 1) {
            // Log.info("LIFT",
            //         "WARNING: Tried to set power above available voltage! Saturation limit SHOULD take care of this");
            output = 1;
        } else if (output < -1) {
            // Log.info("LIFT",
            //         "WARNING: Tried to set power above available voltage! Saturation limit SHOULD take care of this ");
            output = -1;
        }

        if (Math.abs(error) < Constants.LiftConstants.HEIGHT_THRESHOLD) {
            plateauCount++;
        } else {
            plateauCount = 0;
        }


        if((desiredTarget == 0) && !getLimitSwitch()) {
            output = Constants.LiftConstants.ZEROING_VELOCITY;
            // Log.info("Lift", "Using ZEROING_POWER to finish zeroing the arm.");
        } else if((desiredTarget == 0) && getLimitSwitch()) {
            output = 0;
            // Log.info("Lift", "In zero position, setting output to 0.");
        }


        liftMotor.set(ControlMode.PercentOutput, output);

        prevError = error;
        
    }
    public double liftFeedForward(double desired) {
        return 0;
    }

}