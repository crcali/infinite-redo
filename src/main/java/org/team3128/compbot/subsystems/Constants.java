package org.team3128.compbot.subsystems;

import org.team3128.common.utility.units.Length;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.test_suite.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import org.team3128.common.utility.datatypes.PIDConstants;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.RobotConstants;

// the superclass is purely for semantic purposes
public class Constants extends RobotConstants {

        public static class GameConstants {
                public static final double SHOOTER_TARGET_HEIGHT = 98.25; // height of target in INCHES
                public static final double BALL_HEIGHT = 9; // height of the ball in centimeters
        }

        public static class MechanismConstants {
                public static final double ENCODER_RESOLUTION_PER_ROTATION = 2048;
                public static final double inchesToMeters = 0.0254;
                public static final double DT = 0.005; // time between update() method calls for mechanisms
        }

        public static class DriveConstants {
                public static final double kDriveInchesPerSecPerNUp100ms = (1000d / 1)
                                * (1 / MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION)
                                * (Constants.DriveConstants.WHEEL_DIAMETER * Math.PI)
                                * Constants.DriveConstants.WHEEL_ROTATIONS_FOR_ONE_ENCODER_ROTATION; // a fairly basic relationship between tangential and rotational speed: NU/100ms * 1000ms/1second * 1/(encoder resolution) * CIRCUM * (relation between encoder rotations and wheel rotations) = in/s
                public static final double kDriveNuToInches = (1
                                / Constants.MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION)
                                * Constants.DriveConstants.WHEEL_DIAMETER * Math.PI
                                * Constants.DriveConstants.WHEEL_ROTATIONS_FOR_ONE_ENCODER_ROTATION;

                public static final NeutralMode DRIVE_IDLE_MODE = NeutralMode.Brake;

                public static final double ENCODER_ROTATIONS_FOR_ONE_WHEEL_ROTATION = 72 / 8; // basically your gearing. Ask Mech for gear teeth number to gear teeth number ratio: 8.3333333

                public static final double WHEEL_ROTATIONS_FOR_ONE_ENCODER_ROTATION = 1
                                / Constants.DriveConstants.ENCODER_ROTATIONS_FOR_ONE_WHEEL_ROTATION;

                public static final int RIGHT_DRIVE_FRONT_ID = 3;
                public static final int RIGHT_DRIVE_MIDDLE_ID = 2;

                public static final int LEFT_DRIVE_FRONT_ID = 1;
                public static final int LEFT_DRIVE_MIDDLE_ID = 0;

                public static final int DRIVE_HIGH_SPEED = 140; // Empirical Max Linear Speed: TBD in/s

                public static final double WHEEL_DIAMETER = 3.55; // effective wheel diameter (measure first then tune this number until distances are accurate)

                public static final double LEFT_SPEEDSCALAR = 1.0; // purely for TELEOP drive (to make sure that when the drive pushes the joystick forward, both sides of the drivetrain are going ROUGHLY the same speed)
                public static final double RIGHT_SPEEDSCALAR = 1.0;// purely for TELEOP drive (to make sure that when the drive pushes the joystick forward, both sides of the drivetrain are going ROUGHLY the same speed)

                public static final double DRIVE_ACCEL_LIMIT = 120; // Ballpark estimates from mech (Be conservative unless you really need the quick auto paths)
                public static final double DRIVE_JERK_LIMIT = 2000; // Ballpark estimates (Be conservative)

                public static double K_AUTO_RIGHT_P = 0.00007; // 0.00065
                public static double K_AUTO_RIGHT_D = 0.000;
                public static double K_AUTO_RIGHT_F = 1 / 145.9150145782 * kDriveInchesPerSecPerNUp100ms; // 1/(consistent max vel of this side of drivetrain in/s) * conversion to NU/s
                public static double K_AUTO_LEFT_P = 0.00007;
                public static double K_AUTO_LEFT_D = 0.000; // 0.0001
                public static double K_AUTO_LEFT_F = 1 / 140.8705712261 * kDriveInchesPerSecPerNUp100ms; // 1/(consistent max vel of this side of drivetrain in/s) * conversion to NU/s
                public static final double K_HOLD_P = 4;

                public static final double kS = 0.178;
                public static final double kV = 0.055;//0.0516;
                public static final double kA = 0.00679;
                public static final double kP = 0.0013;

        }

        public static class AutonomousDriveConstants {
                public static final double TRACK_RADIUS = 23.89231390411386;
                public static final double MIN_TURNING_RADIUS = 40;
                public static final double MIN_PATH_SPEED = 20;
                public static final double MAX_PATH_SPEED = 120;
                public static final double MIN_LOOKAHEAD_DISTANCE = 14;
                public static final double MAX_LOOKAHEAD_DISTANCE = 30;
                public static final double MAX_TURN_ERROR = 2;
                public static final double MAX_PID_STOP_SPEED = 8;
        }

        public static class VisionConstants {
                public static final double BOTTOM_LIMELIGHT_HEIGHT = 28.85 * Length.in;
                public static final double PIVOT_HEIGHT = 16.0;
                public static final double BOTTOM_LIMELIGHT_ANGLE = 30.0 * Angle.DEGREES;
                public static final double BOTTOM_LIMELIGHT_DISTANCE_FROM_FRONT = 1 * Length.in;
                public static final int SAMPLE_RATE = 3;
                public static final double TX_THRESHOLD = 2; // the maximum error in tx where the shooter will be allowed to shoot
                public static final double TX_OFFSET = 0; // to offset alignment in either direction
                public static final PIDConstants VISION_PID = new PIDConstants(0, 0.014, 0.02, 0.00006);
                public static final PIDConstants BALL_PID = new PIDConstants(0.57, 0.02, 0.0, 0.00003);
                public static final PIDConstants BLIND_BALL_PID = new PIDConstants(0.23, 0, 0, 0);
        }


        public static class TestSuiteConstants {

                public static CanDevices intake;
                public static CanDevices rightDriveLeader;
                public static CanDevices rightDriveFollower;
                public static CanDevices feeder;
                public static CanDevices PDP;
                public static CanDevices leftDriveLeader;
                public static CanDevices leftDriveFollower;
                public static CanDevices armLeader;
                public static CanDevices armFollower;
                public static CanDevices shooterLeft;
                public static CanDevices gatekeeper;
                public static CanDevices shooterRight;
                public static CanDevices corner;
                
        }

        public static class ShooterConstants {
                public static final int SHOOTER_MOTOR_LEFT_ID = 8;
                public static final int SHOOTER_MOTOR_RIGHT_ID = 7;

                public static final double SHOOTER_GEARING = 1.5; // for every 1 rotation of the motor, the shooter does {SHOOTER_GEARING} rotations
                public static final PIDConstants SHOOTER_PID = new PIDConstants(0, 0.000007, 0, 0);

                public static final double SHOOTER_SATURATION_LIMIT = 5; // set limit on integral accumulation (in this case, 1 volt)
                public static final double RPM_THRESHOLD = 100; // the maximum difference between an RPM and the setpoint for a data point to be considered as a plataeu
                public static final int PLATEAU_COUNT = 25; // 50 * 0.005s = 0.25 seconds of plateau required
        }

        public static class HopperConstants {

                public static final int HOPPER_FEEDER_MOTOR_ID = 6;
                public static final int CORNER_MOTOR_ID = 10;
                public static final int GATEKEEPER_MOTOR_ID = 9;

                public static final int SENSOR_0_ID = 8;
                public static final int SENSOR_1_ID = 9;
                //public static final int SENSOR_2_ID = 999;


                public static final double DEBUG_MOTOR_POWER = 0.4;
                public static final int CAPACITY = 5; // num of balls that the hopper can store
                public static final double BASE_POWER = -0.5;
                public static final double FEEDER_REVERSE = 0.25;
                public static final double GATEKEEPER_POWER = -0.75;
                public static final double INDEXER_POWER = -0.40;
                public static final double[] BALL_SPACING = {-6/*25*/, -8/*-27.5*/, -1, 0};//{ -25, -20, -20, -20 }; //offsets for the spacing out the balls in the hopper
                public static final double REVERSE_TIMEOUT = 1.5; //timeout for how long to run the corner motors back (in case the sensor gets triggered)
                public static final int JAM_COUNT_THRESHOLD = 100; //how long the motor has to be stopped in order for it to be considered a jam: 100 loop counts * 1/200hz = 0.5 seconds
                public static final int JAM_REVERSE_COUNT_THRESHOLD = 100; //how long to reverse the intake after a jam is detected: 100 loop counts * 1/200hz = 0.5 seconds
        }

        public static class IntakeConstants {
                public static final int INTAKE_MOTOR_ID = 5;

                public static final double INTAKE_MOTOR_ON_VALUE = -0.65;
                public static final double INTAKE_MOTOR_OFF_VALUE = 0;
                public static final double INTAKE_MOTOR_REVERSE_VALUE = 0.4; //for clearing jams
        }
        
        public static class ClimberConstants {
                public static final int LEFT_MOTOR_ID = 12;
                public static final int RIGHT_MOTOR_ID = 11;
                public static final double CLIMB_POWER = 0.5;
        }

        public static class ArmConstants {
                public static final int ARM_MOTOR_LEADER_ID = 13;
                public static final int ARM_MOTOR_FOLLOWER_ID = 4;
                public static final int ARM_LIMIT_SWITCH_ID = 0;

                public static final NeutralMode ARM_NEUTRAL_MODE = NeutralMode.Brake;
                public static final NeutralMode ARM_NEUTRAL_MODE_DEBUG = NeutralMode.Coast;
                public static final double MAX_ARM_ANGLE = 80;
                public static final double ANGLE_THRESHOLD = 4;
                public static final double PLATEAU_THRESHOLD = 50;

                public static final double ARM_GEARING = 60 / 12 * 80 / 18 * 64 / 8; // for every (ARM_GEARING)
                                                                                     // rotations of the
                // motor, we get 1 rotation of the arm (ask
                // mech for this)
                public static final double ARM_LENGTH = 30; 
                public static final double LIMELIGHT_ARM_LENGTH = 19;
                public static final double LIMELIGHT_ARM_ANGLE = 40.26;

                public static final PIDConstants ARM_PID = new PIDConstants(0, 0.15, 0.01, 0);

                public static final double ARM_SATURATION_LIMIT = 2 / ARM_PID.kI; // set limit on integral accumulation
                public static final double ZEROING_POWER = -0.35;

                public static final double STARTING_ANGLE = 48.87; //the angle that the arm is at when it is within frame perimeter
                public static final int STARTING_POSITION = (int) ((STARTING_ANGLE / 360) * MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION * ARM_GEARING);
        }

}