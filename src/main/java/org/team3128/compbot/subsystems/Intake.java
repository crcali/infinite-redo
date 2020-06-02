package org.team3128.compbot.subsystems;

import org.team3128.common.generics.Threaded;

import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.utility.Log;
import edu.wpi.first.wpilibj.command.Command;

public class Intake extends Threaded {
    public enum IntakeState
    {
        INTAKE_BOX(false, "Intake"),
        OUTTAKE_BOX(false, "Outtake"),
        HOLDING_BOX(false, "Holding"),
        INTAKE_SMARTBELL(false, "Intake"),
        OUTTAKE_SMARTBELL(false, "Outtake"),
        HOLDING_SMARTBELLfalse, "Holding");
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
    public LazyCANSparkMax intakeMotorBoxTop, intakeMotorBoxBottom, intakeMotorSmartBellTop, intakeMotorSmartBellBottom;
    private IntakeState newState;
    public IntakeState currentState;
    private Piston Piston;
    private Thread cargoThread;
    private static Intake instance = null;
    public static Intake getInstance() {
    
        if (instance != null) {
        return instance;
    }
    Log.fatal("LiftIntake", "Attempted to get instance before initializtion! Call initialize(...) first.");
    return null;
    }

    public static void initialize(LazyCANSparkMax intakeMotorTop, LazyCANSparkMax intakeMotorBottom, IntakeState state, Piston Piston) {
        instance = new Intake(intakeMotorTop, intakeMotorBottom, state, Piston);
    }

    private Intake(LazyCANSparkMax intakeMotorTop, LazyCANSparkMax intakeMotorBottom, IntakeState state, Piston Piston) {
        this.intakeMotorTop = intakeMotorTop;
        this.intakeMotorBottom = intakeMotorBottom;
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
        intakeMotorBoxTop.set(power);
        intakeMotorBoxBottom.set(power);
    }
    
    private void setIntakePowerSmartBell(double power) {
        intakeMotorSmartBellTop.set(power);
        intakeMotorSmartBellBottom.set(power);
    }

    @Override
    public void update() {    
        
    }

}