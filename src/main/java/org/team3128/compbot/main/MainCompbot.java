package org.team3128.compbot.main;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.team3128.common.generics.RobotConstants;

import com.kauailabs.navx.frc.AHRS;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.control.trajectory.TrajectoryGenerator;
import org.team3128.common.control.trajectory.constraint.TrajectoryConstraint;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.gyroscope.NavX;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;
import org.team3128.common.vision.CmdHorizontalOffsetFeedbackDrive;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.POVValue;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.listener.controltypes.POV;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.test_suite.CanDevices;
import org.team3128.common.utility.test_suite.ErrorCatcherUtility;
import org.team3128.compbot.subsystems.*;
import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.RobotTracker;
//import org.team3128.compbot.subsystems.StateTracker.RobotState;

import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.Lift.LiftState;
import org.team3128.compbot.subsystems.Intake.IntakeState;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.concurrent.*;

import org.team3128.common.generics.ThreadScheduler;

public class MainCompbot extends NarwhalRobot {

    static Arm arm = Arm.getInstance();
    static Intake intake = Intake.getInstance();
    static Lift lift = Lift.getInstance();

    ExecutorService executor = Executors.newFixedThreadPool(6);
    ThreadScheduler scheduler = new ThreadScheduler();
    Thread auto;

    static FalconDrive drive = FalconDrive.getInstance();

    public Joystick joystickRight, joystickLeft;
    public ListenerManager listenerLeft, listenerRight;
    public AHRS ahrs;
    public static PowerDistributionPanel pdp;

    public NetworkTable table;
    public NetworkTable limelightTable;

    public Limelight limelight;
    public Limelight[] limelights;

    private DriveCommandRunning driveCmdRunning;
    
    public static void setCanChain() {  
        
    }

    @Override
    protected void constructHardware() {
        scheduler.schedule(drive, executor);
        scheduler.schedule(arm, executor);
        scheduler.schedule(intake, executor);
        scheduler.schedule(lift, executor);

        ahrs = drive.ahrs;

        joystickRight = new Joystick(1);
        listenerRight = new ListenerManager(joystickRight);
        addListenerManager(listenerRight);

        joystickLeft = new Joystick(0);
        listenerLeft = new ListenerManager(joystickLeft);
        addListenerManager(listenerLeft);

        driveCmdRunning = new DriveCommandRunning();

        limelight = new Limelight("limelight", Constants.VisionConstants.LIMELIGHT_ANGLE, Constants.VisionConstants.LIMELIGHT_HEIGHT, Constants.VisionConstants.LIMELIGHT_DISTANCE_FROM_FRONT, 14.5 * Length.in);
        limelights = new Limelight[1];
        
    }

    @Override
    protected void constructAutoPrograms() {
        // didn't have enough time to work on auto programs
    }

    @Override
    protected void setupListeners() {

        listenerRight.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
		listenerRight.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
		listenerRight.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");
        listenerRight.addMultiListener(() -> {
            if (driveCmdRunning.isRunning) {
                double horiz = -0.5 * listenerRight.getAxis("MoveTurn"); //0.7
                double vert = -1.0 * listenerRight.getAxis("MoveForwards");
                double throttle = -1.0 * listenerRight.getAxis("Throttle");

                drive.arcadeDrive(horiz, vert, throttle, true);
            }
        }, "MoveTurn", "MoveForwards", "Throttle");
        

        listenerRight.nameControl(new POV(0), "IntakePOV");
        listenerRight.addListener("IntakePOV", (POVValue pov) -> {
            switch (pov.getDirectionValue()) {
                case 8:
                case 7:
                case 1:
                    intake.setState(IntakeState.INTAKE_BOX);
                    break;
                    
                case 3:
                case 4:
                case 5:
                    intake.setState(IntakeState.OUTTAKE_BOX);

                    break;
                    
                case 0:
                    intake.setState(IntakeState.HOLDING_BOX);

                    break;
                default:
                    break;
            }
        });

        listenerRight.nameControl(ControllerExtreme3D.TRIGGER, "armVertical");
        listenerRight.addButtonDownListener("armVertical", () -> {
            arm.setState(ArmState.VERTICAL);
        });

        listenerRight.nameControl(new Button(1), "intakeSmartbell");
        listenerRight.addButtonDownListener("intakeSmartbell", () -> {
            intake.setState(IntakeState.INTAKE_SMARTBELL);
        });

        listenerRight.nameControl(new Button(2), "outtakeSmartbell");
        listenerRight.addButtonDownListener("intakeSmartbell", () -> {
            intake.setState(IntakeState.OUTTAKE_SMARTBELL);
        });

        listenerRight.nameControl(new Button(3), "liftHigh");
        listenerRight.addButtonDownListener("intakeSmartbell", () -> {
            lift.setState(LiftState.TOP);
        });

        listenerRight.nameControl(new Button(3), "liftMiddle");
        listenerRight.addButtonDownListener("intakeSmartbell", () -> {
            lift.setState(LiftState.MIDDLE);
        });

        listenerRight.nameControl(ControllerExtreme3D.JOYY, "MoveLift");
        listenerLeft.addMultiListener(() -> {
            if (driveCmdRunning.isRunning) {
                double vert = -1.0 * listenerLeft.getAxis("MoveLift");

                lift.positionControl(vert);
            }
        }, "MoveLift");
        
    }

    @Override
    protected void teleopPeriodic() {
        scheduler.resume();
    }

    @Override
    protected void updateDashboard() {
        NarwhalDashboard.put("time", DriverStation.getInstance().getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
    }

    @Override
    protected void teleopInit() {
        scheduler.resume();
        driveCmdRunning.isRunning = true;
        limelight.setLEDMode(LEDMode.OFF);
        arm.zero();
        lift.zero();
    }

    @Override
    protected void autonomousInit() {
        scheduler.resume();
        drive.resetGyro();
        arm.zero();
        lift.zero();
    }

    @Override
    protected void disabledInit() {
        lift.liftMotor.setNeutralMode(NeutralMode.Brake);
        limelight.setLEDMode(LEDMode.OFF);
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainCompbot::new);
    }

    @Override
    public void endCompetition() {
        // TODO Auto-generated method stub

    }
}