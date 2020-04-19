package org.team3128.common.utility.test_suite;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.utility.Log;

import org.team3128.common.utility.test_suite.CanDevices;

import com.revrobotics.CANError;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.platform.DeviceType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightKey;
import org.team3128.common.hardware.motor.LazyTalonFX;
import edu.wpi.first.wpilibj.Timer;
//import org.team3128.athos.main.MainAthos;
import org.team3128.common.drive.DriveSignal;
//import org.team3128.athos.subsystems.*;
import org.team3128.common.drive.Drive;

/**
 * Utility used to catch breaks in the CAN chain
 * 
 * @author Tyler Costello, Daniel Wang, Jude T. Lifset
 * 
 */
public class ErrorCatcherUtility {

    public CanDevices[] CanChain = new CanDevices[42];
    public Limelight[] limelights = new Limelight[5];
    
    //CAN Check
    public static ErrorCode errorCode;
    public CanDevices lastDevice;
    private double pdpTemp;
    private double sparkTemp;
    public CANError canError;

    //Drive velocity check
    public boolean forwardWorks;
    public boolean backwardWorks;
    public Drive drive;
    public CANEncoder canEncoder;
    public CanDevices[] driveLeaders = new CanDevices[2];

    DriveSignal driveSignal = new DriveSignal(40, 40); //Can be changed
    DriveSignal backwardsDriveSignal = new DriveSignal(-40, -40);
    DriveSignal zeroDriveSignal = new DriveSignal(0, 0);
    
    public ErrorCatcherUtility(CanDevices[] CanChain, Limelight[] limelights, Drive drive){
      this.CanChain = CanChain;  
      this.limelights = limelights;
      this.drive = drive;
    }

    public void ErrorCatcherCAN(){

        //bridge checker
        NarwhalDashboard.put("ErrorCatcherBridge", "Bridge is connected");


        //Iterates over each CAN device in the chain, in order, and checks if it is good
        errorCode=ErrorCode.OK;
        //for(int i=0;i<CanChain.length;i++){
        //for(int i=12; i>=0;i--){
            //device=CanChain[i];
        for(CanDevices device : CanChain){
            
            if (device == null){
                break;
            }

            if(device.name == "Left Drive Leader")  
                driveLeaders[0] = device;
            if(device.name == "Right Drive Leader")  
                driveLeaders[1] = device;
            
            if(device.type == CanDevices.DeviceType.TALON){
                errorCode = device.talon.configRemoteFeedbackFilter(device.id, RemoteSensorSource.CANifier_Quadrature,0, 10);
               // Log.info("ErrorCatcher", "Talon" +device.id);
            }

            else if (device.type==CanDevices.DeviceType.VICTOR){
                errorCode = device.victor.configRemoteFeedbackFilter(device.id, RemoteSensorSource.CANifier_Quadrature,0, 10);
               // Log.info("ErrorCatcher", "Victor" +device.id);
            }

            else if (device.type==CanDevices.DeviceType.SPARK){

                sparkTemp=device.spark.getMotorTemperature();
                Log.info("ErrorCatcher", "Spark" +device.id+" "+sparkTemp+" degrees");
                //Log.info("ErrorCatcher", "Spark temp "+sparkTemp);

                if (sparkTemp < 5 || sparkTemp>100){
                    errorCode = ErrorCode.CAN_MSG_NOT_FOUND;
                } 
                else if (device.spark.getEncoder() == null){
                    errorCode = ErrorCode.SensorNotPresent;
                } 
                else {
                    errorCode=ErrorCode.OK;
                }

            }

            else if (device.type==CanDevices.DeviceType.FALCON){
               // Log.info("ErrorCatcher", "Falcon" +device.id);
                errorCode = device.falcon.configRemoteFeedbackFilter(device.id, RemoteSensorSource.CANifier_Quadrature,0, 10);
            }

            else if (device.type==CanDevices.DeviceType.PDP){

                pdpTemp=device.pdp.getTemperature();
                //Log.info("ErrorCatcher", "PDP temp "+pdpTemp);

                if (pdpTemp < 5){
                    errorCode = ErrorCode.CAN_MSG_NOT_FOUND;
                }

            }

            //If the current CAN device is not good, log it
            if(errorCode != ErrorCode.OK){
                Log.info("ErrorCatcher", "ErrorCode: "+errorCode);

                if (errorCode == ErrorCode.CAN_MSG_NOT_FOUND || errorCode == ErrorCode.SigNotUpdated){
                    if(device == CanChain[0]){
                        Log.info("ErrorCatcher", "RoboRIO to " +device.name+ " " + device.id +" CAN wire is disconnected");
                        NarwhalDashboard.put("ErrorCatcherCAN", "RoboRIO to " +device.name+ " " + device.id +" CAN wire is disconnected");
                    }

                    else{
                        Log.info("ErrorCatcher", lastDevice.name + " " + lastDevice.id + " to " +device.name+ " " + device.id +" CAN wire is disconnected");
                        NarwhalDashboard.put("ErrorCatcherCAN", lastDevice.name + " " + lastDevice.id + " to " +device.name+ " " + device.id +" CAN wire is disconnected");
                    }

                    //break;
                }

                if (errorCode == ErrorCode.SensorNotPresent){
                    Log.info("ErrorCatcher", device.name+ " " + device.id +" Encoder is disconnected");
                    NarwhalDashboard.put("ErrorCatcherEncoder", device.name+ " " + device.id +" Encoder is disconnected");
                }
                
            } 

            else{
               NarwhalDashboard.put("ErrorCatcherCAN", "No CAN Errors");
            }

            lastDevice = device; 
            //Used because we need the last CAN device in the chain to find which 2 CAN devices are connected by the bad chain

        }
    }

    public void limelightCheck() {
        String limelightError = "";

        for(Limelight limelight : limelights){
            double tempLatency = limelight.getValue(LimelightKey.LATENCY, 5);
            Log.info("ErrorCatcher", "Limelight Latency: " + String.valueOf(tempLatency));

            if(tempLatency == 0){
                Log.info("ErrorCatcher", limelight.hostname + " is disconnected.");
                limelightError = limelightError + "" + limelight.hostname + "is disconnected.\n ";
                //NarwhalDashboard.put("ErrorCatcherLimelight", limelight.hostname + " is disconnected.");  
            } else{
                Log.info("ErrorCatcher", limelight.hostname + " is connected.");
                limelightError = limelightError + "" + limelight.hostname + "is connected.\n ";
                //NarwhalDashboard.put("ErrorCatcherLimelight", limelight.hostname + " is connected.");  
            }
        }
        NarwhalDashboard.put("ErrorCatcherLimelight", limelightError);
    }
    


    public void velocityTester() {
        
        double maxAchieved = 0;
        forwardWorks = false;
        backwardWorks = false;

        double leftEncoderVelocity;
        double rightEncoderVelocity;

        //Forward Movement
        drive.setWheelVelocity(driveSignal);
        double time = Timer.getFPGATimestamp();
        double endTime = Timer.getFPGATimestamp();

       

        leftEncoderVelocity = getEncoderVelocity(driveLeaders[0]);
        rightEncoderVelocity = getEncoderVelocity(driveLeaders[1]);
        while (leftEncoderVelocity < 500 && rightEncoderVelocity < 500 && (endTime-time) <= 1){
            leftEncoderVelocity = getEncoderVelocity(driveLeaders[0]);
            rightEncoderVelocity = getEncoderVelocity(driveLeaders[1]);
           
            maxAchieved = Math.max(maxAchieved, Math.min(leftEncoderVelocity, rightEncoderVelocity));
            endTime = Timer.getFPGATimestamp();
        }

        drive.setWheelVelocity(zeroDriveSignal);

        if (maxAchieved > 50){
            forwardWorks = true;
        }

        Log.info("ErrorCatcher", "Max Velocity "+maxAchieved);
        Log.info("ErrorCatcher", "Movement works "+forwardWorks);

        //Backwards Movement
        drive.setWheelVelocity(backwardsDriveSignal);
        time = Timer.getFPGATimestamp();
        endTime = Timer.getFPGATimestamp();
        maxAchieved = 0;

        leftEncoderVelocity = getEncoderVelocity(driveLeaders[0]);
        rightEncoderVelocity = getEncoderVelocity(driveLeaders[1]);
        Log.info("ErrorCatcher", "Left Encoder velocity: " + leftEncoderVelocity);
        Log.info("ErrorCatcher", "Right Encoder velocity: " + rightEncoderVelocity);

        while (leftEncoderVelocity > -500 && rightEncoderVelocity > -500 && (endTime-time) <= 1){
            leftEncoderVelocity = getEncoderVelocity(driveLeaders[0]);
            rightEncoderVelocity = getEncoderVelocity(driveLeaders[1]);

            //Set maxAchieved to the minimum of the slowest motor and the previous minimum
            maxAchieved = Math.min(maxAchieved, Math.max(leftEncoderVelocity, rightEncoderVelocity)); 

            endTime = Timer.getFPGATimestamp();

        }

        drive.setWheelVelocity(zeroDriveSignal);

        if (maxAchieved < -50){
            backwardWorks = true;
        }
        else {
            backwardWorks = false;
        }
        Log.info("ErrorCatcher", "Left Encoder velocity: " + leftEncoderVelocity);
        Log.info("ErrorCatcher", "Right Encoder velocity: " + rightEncoderVelocity);
        Log.info("ErrorCatcher", "Max Velocity "+maxAchieved);
        Log.info("ErrorCatcher", "Forward Movement works "+forwardWorks);
        Log.info("ErrorCatcher", "Backward Movement works "+backwardWorks);

        if (forwardWorks && backwardWorks){
            NarwhalDashboard.put("ErrorCatcherMovement", "Movement Works");
            Log.info("ErrorCatcher", "Movement Works");  
        }
        else if(forwardWorks && !backwardWorks){
            NarwhalDashboard.put("ErrorCatcherMovement", " Backward Movement Does Not Work");
            Log.info("ErrorCatcher", "Only forward movement works");  
        }
        else if(!forwardWorks && backwardWorks){
            NarwhalDashboard.put("ErrorCatcherMovement", "Forward Movement Does Not Work");
            Log.info("ErrorCatcher", "Only backward movement works");  
        }
        else{
            if (leftEncoderVelocity>=-20 && rightEncoderVelocity<=-20){
                NarwhalDashboard.put("ErrorCatcherMovement", "Movement does not work in either direction. Left Side is broken");
            }
            else if (rightEncoderVelocity>=-20 && leftEncoderVelocity <=-20){
                NarwhalDashboard.put("ErrorCatcherMovement", "Movement does not work in either direction. Right Side is broken");   
            }
            else if (rightEncoderVelocity>=-20 && leftEncoderVelocity >=-20){
                NarwhalDashboard.put("ErrorCatcherMovement", "Is it enabled? Movement does not work in either direction. Both Sides are broken.");   
            }
            else {
                NarwhalDashboard.put("ErrorCatcherMovement", "Movement does not work in either direction");
            }
            Log.info("ErrorCatcher", "Movement does not work in either direction");
        }
    }
    public double getEncoderVelocity(CanDevices canDevice){
        switch(canDevice.type){
            case TALON:
                return canDevice.talon.getSelectedSensorVelocity();
            case FALCON:
                return  canDevice.falcon.getSelectedSensorVelocity();
            case SPARK:
                return canDevice.spark.getEncoder().getVelocity();
            default:
                return 0; 
        }
        
    }
    
    public void testEverything() {
        //janky fix
        ErrorCatcherCAN();
        ErrorCatcherCAN();
        limelightCheck();
        //velocityTester();
    }
}