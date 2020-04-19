package org.team3128.common.drive;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.team3128.common.generics.RobotConstants;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.control.trajectory.TrajectoryGenerator;
import org.team3128.common.control.trajectory.constraint.TrajectoryConstraint;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.common.hardware.gyroscope.NavX;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;
import org.team3128.common.vision.CmdHorizontalOffsetFeedbackDrive;
import org.team3128.athos.subsystems.Constants;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.athos.subsystems.NEODrive;
import org.team3128.athos.subsystems.RobotTracker;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.concurrent.*;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

import org.team3128.common.generics.ThreadScheduler;

import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.motor.LazyTalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import org.team3128.common.drive.DriveSignal;
import org.team3128.common.utility.test_suite.*;
import org.team3128.common.drive.Drive;

import org.team3128.common.generics.Threaded;
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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.utility.RobotMath;

import edu.wpi.first.wpilibj.Timer;


/** 
 * @author Tyler Costello and Daniel Wang 
 * Abstract class for drivetrain objects to inherit
 */

import org.team3128.common.utility.Log;
public abstract class Drive extends Threaded {

    public abstract void debug();

    public abstract void debugSpeed();

    public abstract void setRight();

    public abstract void configAuto();

    public abstract void configHigh();

    public abstract void setTeleop(); //should be synchronized

    public abstract void calibrateGyro();

    public abstract void printCurrent();

    public abstract void configMotors();

    public abstract void resetMotionProfile();

    public abstract double getAngle();

    public abstract double getDistance();

    public abstract Rotation2D getGyroAngle();

    public abstract double getLeftDistance();

    public abstract double getRightDistance();

    public abstract double getSpeed();

    public abstract double getLeftSpeed();

    public abstract double getRightSpeed();

    public abstract void setAutoTrajectory(Trajectory autoTraj, boolean isReversed); // synchronized

    public abstract void startTrajectory(); // synchronized

    public abstract void setBrakeState(NeutralMode mode);

    public abstract double getVoltage();

    public abstract void setWheelPower(DriveSignal signal);

    public abstract void setWheelVelocity(DriveSignal setVelocity);

    /**
     * Update the motor outputs with the given control values.
     *
     * @param joyX     horizontal control input
     * @param joyY     vertical control input
     * @param throttle throttle control input scaled between 1 and -1 (-.8 is 10 %,
     *                 0 is 50%, 1.0 is 100%)
     */
    public abstract void arcadeDrive(double joyX, double joyY, double throttle, boolean fullSpeed);

    @Override
    public abstract void update();

    public abstract void setRotation(Rotation2D angle);

    /**
     * Set Velocity PID for both sides of the drivetrain (to the same constants)
     */
    public abstract void setDualVelocityPID(double kP, double kD, double kF);

    public abstract void updateTurn();

    public abstract void setShiftState(boolean state);

    public abstract void updateRamseteController(boolean isStart);

    public abstract void resetGyro();

    public abstract boolean checkSubsystem();

    public abstract void stopMovement(); // synchronized

    public abstract boolean isFinished(); // synchronized

    public abstract void clearStickyFaults();

}