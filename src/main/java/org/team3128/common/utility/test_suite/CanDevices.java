package org.team3128.common.utility.test_suite;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team3128.common.hardware.motor.LazyTalonSRX;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

import org.team3128.common.hardware.motor.LazyCANSparkMax;

import org.team3128.common.hardware.motor.LazyTalonFX;

/**
 * This class is used to create CAN device objects.
 * 
 * @author Tyler Costello, Daniel Wang, Jude T. Lifset
 *
 */
public class CanDevices{
    
    public enum DeviceType
    { 
        VICTOR,
        TALON,
        SPARK,
        FALCON,
        PDP;
    } 
    public DeviceType type;
    public int id;
    public String name;
    public LazyTalonSRX talon;
    public VictorSPX victor;
    public PowerDistributionPanel pdp;
    public LazyCANSparkMax spark;
    public LazyTalonFX falcon;
    

   /* public CanDevices(int id, String name, LazyTalonSRX talon, VictorSPX victor, LazyCANSparkMax spark, LazyTalonFX falcon, PowerDistributionPanel pdp){
        this.type = type;
        this.id = id;
        this.name = name;
        this.talon = talon;
        this.victor = victor;
        this.spark = spark;
        this.falcon = falcon;
        this.pdp = pdp;

    }*/
    public CanDevices(int id, String name, LazyTalonSRX talon){
        this.type = DeviceType.TALON;
        this.id = id;
        this.name = name;
        this.talon = talon;
    }
    public CanDevices(int id, String name, VictorSPX victor){
        this.type = DeviceType.VICTOR;
        this.id = id;
        this.name = name;
        this.victor = victor;
    }
    public CanDevices(int id, String name, LazyCANSparkMax spark){
        this.type = DeviceType.SPARK;
        this.id = id;
        this.name = name;
        this.spark = spark;
    }
    public CanDevices(int id, String name, LazyTalonFX falcon){
        this.type = DeviceType.FALCON;
        this.id = id;
        this.name = name;
        this.falcon = falcon;
    }
    public CanDevices(int id, String name, PowerDistributionPanel pdp){
        this.type = DeviceType.PDP;
        this.id = id;
        this.name = name;
        this.pdp = pdp;
    }
    

   

}