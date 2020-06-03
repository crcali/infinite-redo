package org.team3128.compbot.subsystems;

import org.team3128.common.generics.Threaded;

import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.utility.Log;
import edu.wpi.first.wpilibj.command.Command;

import org.team3128.compbot.subsystems.*;

public class Intake extends Threaded {
    public enum IntakeState
    {
        INTAKE_BOX(false, "Intake"),
        OUTTAKE_BOX(false, "Outtake"),
        HOLDING_BOX(false, "Holding"),
        INTAKE_SMARTBELL(false, "Intake"),
        OUTTAKE_SMARTBELL(false, "Outtake"),
        HOLDING_SMARTBELL(false, "Holding");
        private boolean PistonState;
        private String name;
        private IntakeState(boolean PistonState, String name) {
            this.PistonState = PistonState;
            this.name = name;
        }
        public boolean getPistonState() {
            return PistonState;
        }
        public String getName() {
            return name;
        }
    }
    public LazyCANSparkMax intakeMotorBox, intakeMotorSmartbell;
    private IntakeState newState;
    public IntakeState currentState;
    private Piston Piston;
    private static Intake instance = null;
    public static Intake getInstance() {
    
        if (instance != null) {
        return instance;
    }
    Log.fatal("LiftIntake", "Attempted to get instance before initializtion! Call initialize(...) first.");
    return null;
    }

    public static void initialize(LazyCANSparkMax intakeMotorBox, LazyCANSparkMax intakeMotorSmartbell, IntakeState state, Piston Piston) {
        instance = new Intake(intakeMotorBox, intakeMotorSmartbell, state, Piston);
    }

    private Intake(LazyCANSparkMax intakeMotorBox, LazyCANSparkMax intakeMotorSmartbell, IntakeState state, Piston Piston) {
        this.intakeMotorBox = intakeMotorBox;
        this.intakeMotorSmartbell = intakeMotorSmartbell;
        this.Piston = Piston;
        this.newState = state;
    }

    public void setState(IntakeState newState) {
        if (this.currentState != newState) {
            if (newState.getPistonState()) {
                Piston.setPistonOn();
            }
            else {
                Piston.setPistonOff();
            }
            this.newState = newState;
        }
    }
    private void setIntakePowerBox(double power) {
        intakeMotorBox.set(power);
    }
    
    private void setIntakePowerSmartBell(double power) {
        intakeMotorSmartbell.set(power);
    }

    @Override
    public void update() {    
        if (this.newState == null) {
            if (Arm.state == Arm.ArmState.DOWN_SMARTBELL) {
                this.setIntakePowerSmartBell(-1.0);
            }

            if (Arm.state == Arm.ArmState.DOWN_BOX) {
                this.setIntakePowerSmartBell(-1.0);
            }
        }
        
        else {
            if (this.newState == IntakeState.INTAKE_BOX) {
            this.setIntakePowerBox(-1.0);
            }
            else if (this.newState == IntakeState.OUTTAKE_BOX) {
                this.setIntakePowerBox(1.0);
            }
            else {
                this.setIntakePowerBox(0.0);
            }

            if (this.newState == IntakeState.INTAKE_SMARTBELL) {
                this.setIntakePowerBox(-1.0);
            }
            else if (this.newState == IntakeState.OUTTAKE_SMARTBELL) {
                this.setIntakePowerSmartBell(1.0);
            }
            else {
                this.setIntakePowerSmartBell(0.0);
            }
        }
    }

}