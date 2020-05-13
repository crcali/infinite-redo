package org.team3128.compbot.subsystems;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.utility.Log;
import edu.wpi.first.wpilibj.command.Command;
public class Intake {
    public enum IntakeState
    {
        INTAKE(true, “Intake”),
        OUTTAKE(true, “Outtake”),
        HOLDING(true, “Holding”);
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
    public LazyCANSparkMax intakeMotorTop, intakeMotorBottom;
    private IntakeState newState;
    public IntakeState currentState;
    private Piston Piston;
    private Thread cargoThread;
    private static Intake instance = null;
    public static Intake getInstance() {
        if (instance != null) {
            return instance;
        }
        Log.fatal(“LiftIntake”, “Attempted to get instance before initializtion! Call initialize(...) first.“);
        return null;
    }
    public static void initialize(LazyCANSparkMax intakeMotorTop, LazyCANSparkMax intakeMotorBottom, IntakeState state, Piston Piston) {
        instance = new Intake(intakeMotorTop, intakeMotorBottom, state, Piston);
    }
    private Intake(LazyCANSparkMax intakeMotorTop, LazyCANSparkMax intakeMotorBottom, IntakeState state, Piston Piston) {
        this.intakeMotorTop = intakeMotorTop;
        this.intakeMotorBottom = intakeMotorBottom;
        this.Piston = Piston;
        setState(state);
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
    private void setIntakePower(double power) {
        intakeMotorTop.set(power);
        intakeMotorBottom.set(power);
    }
    public class CmdSetLiftIntakeState extends Command {
        IntakeState desiredState;
        public CmdSetLiftIntakeState(IntakeState state) {
            super(0.1);
            desiredState = state;
        }
        @Override
        protected void initialize()
        {
            setState(desiredState);
        }
        @Override
        protected boolean isFinished()
        {
            return isTimedOut();
        }
    }
}

