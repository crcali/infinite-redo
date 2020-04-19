/** 
 * @author Adham Elarabawy 
 */
package org.team3128.athos.subsystems;

import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.generics.Threaded;
import org.team3128.common.control.RateLimiter;
import org.team3128.common.control.AsynchronousPid;
import org.team3128.common.control.motion.RamseteController;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.control.trajectory.Trajectory.State;
import org.team3128.common.drive.AutoDriveSignal;
import org.team3128.common.drive.DriveSignal;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.NarwhalUtility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.utility.RobotMath;

import edu.wpi.first.wpilibj.Timer;

import org.team3128.common.utility.Log;
import org.team3128.common.drive.Drive;

public class NEODrive extends Drive {

	public enum DriveState {
		TELEOP, RAMSETECONTROL, TURN, DONE
	}

	private static final NEODrive instance = new NEODrive();

	public static NEODrive getInstance() {
		return instance;
	}

	private ADXRS450_Gyro gyroSensor;
	private RamseteController autonomousDriver;
	private Trajectory trajectory = null;
	private AsynchronousPid turnPID;
	private DriveState driveState;
	private RateLimiter moveProfiler, turnProfiler;
	private Rotation2D wantedHeading;
	private volatile double driveMultiplier;

	private double currentTime;
	private double startTime;
	private double totalTime;

	double prevPositionL = 0;
	double prevPositionR = 0;

	public static LazyCANSparkMax leftSpark, rightSpark, leftSparkSlave, rightSparkSlave, leftSparkSlave2, rightSparkSlave2;
	private CANPIDController leftSparkPID, rightSparkPID;
	private CANEncoder leftSparkEncoder, rightSparkEncoder;

	private NEODrive() {

		gyroSensor = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

		leftSpark = new LazyCANSparkMax(Constants.LEFT_DRIVE_FRONT_ID, MotorType.kBrushless);
		leftSparkSlave = new LazyCANSparkMax(Constants.LEFT_DRIVE_MIDDLE_ID, MotorType.kBrushless);
		// leftSparkSlave2 = new LazyCANSparkMax(Constants.LEFT_DRIVE_BACK_ID,
		// MotorType.kBrushless);

		rightSpark = new LazyCANSparkMax(Constants.RIGHT_DRIVE_FRONT_ID, MotorType.kBrushless);
		rightSparkSlave = new LazyCANSparkMax(Constants.RIGHT_DRIVE_MIDDLE_ID, MotorType.kBrushless);
		// rightSparkSlave2 = new LazyCANSparkMax(Constants.RIGHT_DRIVE_BACK_ID,
		// MotorType.kBrushless);

		leftSpark.setInverted(true);
		rightSpark.setInverted(false);
		leftSparkSlave.setInverted(true);
		rightSparkSlave.setInverted(false);
		// leftSparkSlave2.setInverted(false);
		// rightSparkSlave2.setInverted(false);

		leftSparkPID = leftSpark.getPIDController();
		rightSparkPID = rightSpark.getPIDController();
		leftSparkEncoder = leftSpark.getEncoder();
		rightSparkEncoder = rightSpark.getEncoder();

		configMotors();

		driveState = DriveState.TELEOP;

		turnPID = new AsynchronousPid(1.0, 0, 1.2, 0); // P=1.0 OR 0.8
		turnPID.setOutputRange(Constants.DRIVE_HIGH_SPEED, -Constants.DRIVE_HIGH_SPEED);
		turnPID.setSetpoint(0);

		moveProfiler = new RateLimiter(Constants.DRIVE_ACCEL_LIMIT);
		turnProfiler = new RateLimiter(100);

		// configHigh();
		configAuto();
	}

	@Override public void debug() {
		System.out.println("L enc: " + getLeftDistance() + " velo " + getLeftSpeed());
		System.out.println("R enc: " + getRightDistance() + " velo " + getRightSpeed());
		System.out.println("Gyro: " + getAngle()/* getGyroAngle().getDegrees() */);
	}

	@Override public void debugSpeed() {
		System.out.println("L speed " + " actual " + getLeftSpeed());
		System.out.println("R speed " + " actual " + getRightSpeed());

	}

	@Override public void setRight() {
		setWheelVelocity(new DriveSignal(40, 0));
	}

	@Override public void configAuto() {
		rightSparkPID.setP(Constants.K_AUTO_RIGHT_P, 0);
		rightSparkPID.setD(Constants.K_AUTO_RIGHT_D, 0);
		rightSparkPID.setFF(Constants.K_AUTO_RIGHT_F, 0);
		rightSparkPID.setOutputRange(-1, 1);

		leftSparkPID.setP(Constants.K_AUTO_LEFT_P, 0);
		leftSparkPID.setD(Constants.K_AUTO_LEFT_D, 0);
		leftSparkPID.setFF(Constants.K_AUTO_LEFT_F, 0);
		leftSparkPID.setOutputRange(-1, 1);

	}

	@Override public void configHigh() {
		driveMultiplier = Constants.DRIVE_HIGH_SPEED;
	}

	boolean teleopstart = true;

	synchronized public void setTeleop() {
		driveState = DriveState.TELEOP;
	}

	@Override public void calibrateGyro() {
		gyroSensor.calibrate();
	}

	@Override public void printCurrent() {
		System.out.println(leftSpark);
	}

	@Override public void configMotors() {
		leftSparkSlave.follow(leftSpark);
		// leftSparkSlave2.follow(leftSpark);
		rightSparkSlave.follow(rightSpark);
		// rightSparkSlave2.follow(rightSpark);

		leftSpark.setIdleMode(Constants.DRIVE_IDLE_MODE);
		rightSpark.setIdleMode(Constants.DRIVE_IDLE_MODE);
		leftSparkSlave.setIdleMode(Constants.DRIVE_IDLE_MODE);
		rightSparkSlave.setIdleMode(Constants.DRIVE_IDLE_MODE);
		// leftSparkSlave2.setIdleMode(IdleMode.kCoast);
		// rightSparkSlave2.setIdleMode(IdleMode.kCoast);
		configAuto();
	}

	@Override public void resetMotionProfile() {
		moveProfiler.reset();
	}

	@Override public double getAngle() {
		return -gyroSensor.getAngle();
	}

	@Override public double getDistance() {
		return (getLeftDistance() + getRightDistance()) / 2;
	}

	@Override public Rotation2D getGyroAngle() {
		// -180 through 180
		return Rotation2D.fromDegrees(gyroSensor.getAngle());
	}

	@Override public double getLeftDistance() {
		/*
		 * return leftTalon.getSelectedSensorPosition(0) /
		 * Constants.EncoderTicksPerRotation * Constants.WheelDiameter Math.PI * 22d /
		 * 62d / 3d;
		 */
		return leftSparkEncoder.getPosition() * Constants.WHEEL_DIAMETER * Math.PI
				* Constants.WHEEL_ROTATIONS_FOR_ONE_ENCODER_ROTATION;
	}

	@Override public double getRightDistance() {
		return rightSparkEncoder.getPosition() * Constants.WHEEL_DIAMETER * Math.PI
				* Constants.WHEEL_ROTATIONS_FOR_ONE_ENCODER_ROTATION;
	}

	@Override public double getSpeed() {
		return (getLeftSpeed() + getRightSpeed()) / 2;
	}

	@Override public double getLeftSpeed() {
		return leftSparkEncoder.getVelocity() * Constants.kDriveInchesPerSecPerRPM;
	}

	@Override public double getRightSpeed() {
		return rightSparkEncoder.getVelocity() * Constants.kDriveInchesPerSecPerRPM;
	}

	@Override public synchronized void setAutoTrajectory(Trajectory autoTraj, boolean isReversed) {
		this.trajectory = autoTraj;
		totalTime = trajectory.getTotalTimeSeconds();
		autonomousDriver = new RamseteController(1.8, 0.7, isReversed, Constants.TRACK_RADIUS); // 2,0.7
	}

	@Override public synchronized void startTrajectory() {
		if (trajectory == null) {
			Log.info("NEODrive", "FATAL // FAILED TRAJECTORY - NULL TRAJECTORY INPUTTED");
			Log.info("NEODrive", "Returned to teleop control");
			driveState = DriveState.TELEOP;
		} else {
			configAuto();
			updateRamseteController(true);
			driveState = DriveState.RAMSETECONTROL;
		}
	}

	@Override public void setBrakeState(NeutralMode mode) {
	}

	@Override public double getVoltage() {
		return 0;
	}

	public void setWheelPower(DriveSignal setVelocity) {
		leftSpark.set(setVelocity.leftVelocity);
		rightSpark.set(setVelocity.rightVelocity);
	}

	public void setWheelVelocity(DriveSignal setVelocity) {
		if (Math.abs(setVelocity.rightVelocity) > (Constants.DRIVE_HIGH_SPEED)
				|| Math.abs(setVelocity.leftVelocity) > (Constants.DRIVE_HIGH_SPEED)) {
			DriverStation.getInstance();
			DriverStation.reportError("Velocity set over " + Constants.DRIVE_HIGH_SPEED + " !", false);
			return;
		}
		// inches per sec to rotations per min
		double leftSetpoint = (setVelocity.leftVelocity) * 1 / Constants.kDriveInchesPerSecPerRPM;
		double rightSetpoint = (setVelocity.rightVelocity) * 1 / Constants.kDriveInchesPerSecPerRPM;
		leftSparkPID.setReference(leftSetpoint, ControlType.kVelocity);
		// Log.info("NEODrive", "setWheelVelocity: " + "leftSetpoint = " +
		// String.valueOf(leftSetpoint));
		rightSparkPID.setReference(rightSetpoint, ControlType.kVelocity);
		// Log.info("NEODrive", "setWheelVelocity: " + "rightSetpoint = " +
		// String.valueOf(rightSetpoint));
	}

	/**
	 * Update the motor outputs with the given control values.
	 *
	 * @param joyX     horizontal control input
	 * @param joyY     vertical control input
	 * @param throttle throttle control input scaled between 1 and -1 (-.8 is 10 %,
	 *                 0 is 50%, 1.0 is 100%)
	 */
	@Override public void arcadeDrive(double joyX, double joyY, double throttle, boolean fullSpeed) {
		synchronized (this) {
			driveState = DriveState.TELEOP;
		}

		double spdL, spdR, pwrL, pwrR;

		if (!fullSpeed) {
			joyY *= .65;
		} else {
			joyY *= 1;
		}

		// scale from 1 to -1 to 1 to 0
		throttle = (throttle + 1) / 2;

		if (throttle < .3) {
			throttle = .3;
		} else if (throttle > .8) {
			throttle = 1;
		}

		joyY *= throttle;
		joyX *= throttle;

		pwrL = Constants.LEFT_SPEEDSCALAR * RobotMath.clampPosNeg1(joyY - joyX);
		pwrR = Constants.RIGHT_SPEEDSCALAR * RobotMath.clampPosNeg1(joyY + joyX);

		spdL = Constants.DRIVE_HIGH_SPEED * pwrL;
		spdR = Constants.DRIVE_HIGH_SPEED * pwrR;
		String tempStr = "pwrL=" + String.valueOf(pwrL) + ", pwrR=" + String.valueOf(pwrR) + ", spdL="
				+ String.valueOf(spdL) + ", spdR=" + String.valueOf(spdR);
		Log.info("NEODrive", tempStr);
		// setWheelPower(new DriveSignal(pwrL, pwrR));
		setWheelVelocity(new DriveSignal(spdL, spdR));
	}


	@Override public void update() {
		// System.out.println("L speed " + getLeftSpeed() + " position x " +
		// RobotTracker.getInstance().getOdometry().translationMat.getX());
		// System.out.println("R speed " + getRightSpeed() + " position y " +
		// RobotTracker.getInstance().getOdometry().translationMat.getY());
		// System.out.println(driveState);
		DriveState snapDriveState;
		synchronized (this) {
			snapDriveState = driveState;
		}
		switch (snapDriveState) {
		case TELEOP:
			break;
		case RAMSETECONTROL:
			updateRamseteController(false);
			break;
		case TURN:
			updateTurn();
			break;
		}
	}

	@Override public void setRotation(Rotation2D angle) {
		synchronized (this) {
			wantedHeading = angle;
			driveState = DriveState.TURN;
		}
		configHigh();
	}

	/**
	 * Set Velocity PID for both sides of the drivetrain (to the same constants)
	 */
	@Override public void setDualVelocityPID(double kP, double kD, double kF) {
		leftSparkPID.setP(kP);
		leftSparkPID.setD(kD);
		leftSparkPID.setFF(kF);

		rightSparkPID.setP(kP);
		rightSparkPID.setD(kD);
		rightSparkPID.setFF(kF);
		Log.info("[NEODrive]", "Updated Velocity PID values for both sides of the drivetrain to: kP = " + kP + ", kD = "
				+ kD + ", kF = " + kF);
	}

	@Override public void updateTurn() {
		double error = wantedHeading.rotateBy(RobotTracker.getInstance().getOdometry().rotationMat.inverse())
				.getDegrees();
		double deltaSpeed;
		// System.out.println(RobotTracker.getInstance().getOdometry().rotationMat.getDegrees());
		// System.out.println("error: " + error);
		deltaSpeed = turnPID.update(error);
		deltaSpeed = Math.copySign(
				NarwhalUtility.coercedNormalize(Math.abs(deltaSpeed), 0, 180, 0, Constants.DRIVE_HIGH_SPEED),
				deltaSpeed);
		if (Math.abs(error) < Constants.MAX_TURN_ERROR && deltaSpeed < Constants.MAX_PID_STOP_SPEED) {
			setWheelVelocity(new DriveSignal(0, 0));
			synchronized (this) {
				driveState = DriveState.DONE;
			}
		} else {
			setWheelVelocity(new DriveSignal(-deltaSpeed, deltaSpeed));
		}
	}

	@Override public void setShiftState(boolean state) {
		configHigh();
	}

	@Override public void updateRamseteController(boolean isStart) {
		currentTime = Timer.getFPGATimestamp();
		if (isStart) {
			startTime = currentTime;
		}
		State currentTrajectoryState = trajectory.sample(currentTime - startTime);

		AutoDriveSignal signal = autonomousDriver.calculate(RobotTracker.getInstance().getOdometry(),
				currentTrajectoryState);
		if ((currentTime - startTime) == totalTime) {
			synchronized (this) {
				Log.info("NEODrive", "Finished Trajectory Pursuit with RamseteController successfully.");
				driveState = DriveState.TELEOP;
			}
			configHigh();
		}
		// System.out.println("signal l:" + signal.command.leftVelocity + " signal R " +
		// signal.command.rightVelocity);
		setWheelVelocity(signal.command);
	}

	@Override public void resetGyro() {
		gyroSensor.reset();
	}

	@Override public boolean checkSubsystem() {
		configMotors();
		return true;
	}

	@Override synchronized public void stopMovement() {
		leftSpark.set(0);
		rightSpark.set(0);
		leftSparkPID.setReference(0, ControlType.kDutyCycle);
		rightSparkPID.setReference(0, ControlType.kDutyCycle);
		setWheelVelocity(new DriveSignal(0, 0));

		driveState = DriveState.TELEOP;
		resetMotionProfile();
	}

	@Override synchronized public boolean isFinished() {
		return driveState == DriveState.DONE || driveState == DriveState.TELEOP;
	}

	@Override public void clearStickyFaults() {
	}
}
