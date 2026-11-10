package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.utilities.Util;

/**
 * SPARK MAX controllers are initialized over CAN by constructing a SparkMax
 * object
 *
 * Updated for REVLib 2026:
 * - ResetMode / PersistMode now come from com.revrobotics (not SparkBase)
 * - setReference() replaced by setSetpoint()
 * - closedLoop.velocityFF() replaced by closedLoop.feedForward.kV() (units are now VOLTS per RPM)
 * - limitSwitch.xxxLimitSwitchEnabled() replaced by xxxLimitSwitchTriggerBehavior()
 * - maxMotion.maxVelocity() replaced by cruiseVelocity()
 * - maxMotion.allowedClosedLoopError() replaced by allowedProfileError()
 *
 * Bug fixes:
 * - The follow motor now actually follows the leader (it was configured but never told to follow)
 * - The invert constructor parameter is now applied
 * - Leader and follower have separate configs, so leader updates (PID, brake, current limit)
 *   no longer get overwritten by the follower's config
 * - Velocity logging reads the NEO's built-in relative encoder instead of the absolute encoder
 *   (the absolute encoder reads 0 when none is attached)
 * - setSpeed() works again after SysId has driven the motor with setVoltage()
 *
 * Look to:
 *
 * https://docs.revrobotics.com/revlib/changelog
 * https://codedocs.revrobotics.com/java/com/revrobotics/spark/package-summary
 * https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control
 */

public class MotorFlex extends SubsystemBase {
    /** Nominal battery voltage used to convert old duty-cycle feedforward gains to volts. */
    private static final double NOMINAL_VOLTAGE = 12.0;

    /**
     * Set true if the follow motor is mounted so it must spin opposite the leader
     * (for example, two motors facing each other on one shaft).
     */
    private static final boolean FOLLOWER_OPPOSES_LEADER = false;

    private SparkMax motor;
    private SparkMax followMotor;
    private String name;
    private CommandXboxController controller;
    private int id;
    private int followId;
    private boolean brakeMode;
    private boolean invert;
    private double lastSpeed;
    private SparkLimitSwitch forwardSwitch;
    private SparkLimitSwitch reverseSwitch;
    private boolean myLogging = true;
    private double lastDesiredPosition;
    private RelativeEncoder relEncoder;
    private RelativeEncoder followEncoder;
    private double lastPos = 0;
    private SparkMaxConfig motorConfig;
    private SparkMaxConfig followConfig;
    private double positionConversionFactor = 1.0;
    private double velocityConversionFactor = 1.0;
    private boolean testMode = false;
    private int numberCyclesForDisplay = 1000000;
    private final SysIdRoutine sysId;
    

    public MotorFlex(String name, int id, int followId, CommandXboxController controller, boolean brakeMode,
            boolean invert) {
        this.controller = controller;
        this.name = name;
        this.id = id;
        this.followId = followId;
        this.brakeMode = brakeMode;
        this.invert = invert;
        myMotorFlex(name, id, this.followId, false);

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        Seconds.of(3.5),
                        (state) -> Logger.recordOutput(name + "/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage), null, this));
    }

    void myMotorFlex(String name, int id, int followId, boolean logging) {
        this.name = name;
        this.id = id;
        this.followId = followId;
        myLogging = logging;
        logf("Start Spark Max %s id:%d\n", name, id);

        motor = new SparkMax(id, MotorType.kBrushless);
        if (followId > 0) {
            followMotor = new SparkMax(followId, MotorType.kBrushless);
        }

        motorConfig = buildLeaderConfig();
        applyConfig(ResetMode.kResetSafeParameters);

        relEncoder = motor.getEncoder();
        relEncoder.setPosition(0.0);
        if (followMotor != null) {
            followEncoder = followMotor.getEncoder();
        }

        forwardSwitch = motor.getForwardLimitSwitch();
        reverseSwitch = motor.getReverseLimitSwitch();

        if (followMotor != null)
            logf("Created %s dual motors ids:<%d,%d> firmware:<%s,%s> \n", name, id,
                    followId, motor.getFirmwareString(), followMotor.getFirmwareString());
        else
            logf("Created %s motor id:%d firmware:%s \n", name, id,
                    motor.getFirmwareString());
    }

    public void setTestMode(boolean value) {
        testMode = value;
    }

    public void setLogging(boolean value) {
        myLogging = value;
    }

    /** Builds the full configuration for the leader motor. */
    private SparkMaxConfig buildLeaderConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40);
        config.inverted(invert);
        config.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        config.limitSwitch
                .forwardLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotor)
                .reverseLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotor);

        // Using the NEO's integrated encoder; adjust conversion factors here if needed.
        config.encoder
                .positionConversionFactor(positionConversionFactor)
                .velocityConversionFactor(velocityConversionFactor);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        // Slot 0: position control
        config.closedLoop
                .p(0.16, ClosedLoopSlot.kSlot0)
                .i(0, ClosedLoopSlot.kSlot0)
                .d(0, ClosedLoopSlot.kSlot0) // was 1
                .outputRange(-1, 1, ClosedLoopSlot.kSlot0);

        // Slot 1: velocity control
        config.closedLoop
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        // Velocity feedforward. Old velocityFF was duty cycle per RPM (1/5767);
        // the new kV is volts per RPM, so scale by nominal voltage.
        // Note 5767 is max speed for Velocity PIDs
        config.closedLoop.feedForward
                .kV(NOMINAL_VOLTAGE / 5767, ClosedLoopSlot.kSlot1);

        // Slot 2: MAXMotion position control
        config.closedLoop
                .p(1, ClosedLoopSlot.kSlot2)
                .i(0, ClosedLoopSlot.kSlot2)
                .d(0, ClosedLoopSlot.kSlot2) // was 1
                .outputRange(-1, 1, ClosedLoopSlot.kSlot2);

        // Slot 3: MAXMotion velocity control
        config.closedLoop
                .p(0.0002, ClosedLoopSlot.kSlot3)
                .i(0, ClosedLoopSlot.kSlot3)
                .d(0, ClosedLoopSlot.kSlot3) // was 1
                .outputRange(-1, 1, ClosedLoopSlot.kSlot3);

        config.closedLoop.maxMotion
                // MAXMotion parameters for slot 1
                .maxAcceleration(3000, ClosedLoopSlot.kSlot1) // Was 500
                .cruiseVelocity(6000, ClosedLoopSlot.kSlot1)
                .allowedProfileError(.1, ClosedLoopSlot.kSlot1)
                // MAXMotion parameters for position control in slot 2
                .maxAcceleration(3000, ClosedLoopSlot.kSlot2) // Was 500
                .cruiseVelocity(6000, ClosedLoopSlot.kSlot2)
                .allowedProfileError(.1, ClosedLoopSlot.kSlot2)
                // MAXMotion parameters for velocity control in slot 3
                .maxAcceleration(3000, ClosedLoopSlot.kSlot3) // Was 500
                .cruiseVelocity(6000, ClosedLoopSlot.kSlot3)
                .allowedProfileError(.1, ClosedLoopSlot.kSlot3);

        return config;
    }

    /**
     * Applies motorConfig to the leader and, if there is one, a matching follower
     * config that follows the leader.
     *
     * kNoPersistParameters means the settings are not saved to the SPARK MAX's flash,
     * so they are re-sent every time the robot code starts. Switch to
     * kPersistParameters if they need to survive a brownout mid-match.
     */
    private void applyConfig(ResetMode resetMode) {
        motor.configure(motorConfig, resetMode, PersistMode.kNoPersistParameters);

        if (followMotor != null) {
            followConfig = new SparkMaxConfig();
            followConfig.apply(motorConfig);
            followConfig.follow(id, FOLLOWER_OPPOSES_LEADER);
            followMotor.configure(followConfig, resetMode, PersistMode.kNoPersistParameters);
        }
    }

    /**
     * Note: pid.kF is treated as the old duty-cycle-per-velocity gain and converted
     * to volts for the new kV feedforward.
     */
    public void putPIDtoMotor(PID pid, ClosedLoopSlot slot) {
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(pid.kP, slot)
                .i(pid.kI, slot)
                .d(pid.kD, slot)
                .iZone(pid.kIz, slot)
                .outputRange(pid.kMinOutput, pid.kMaxOutput, slot);
        motorConfig.closedLoop.feedForward
                .kV(pid.kF * NOMINAL_VOLTAGE, slot);
        // PID only runs on the leader; the follower just mirrors its output.
        motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setCurrentLimit(int limit) {
        motorConfig.smartCurrentLimit(limit);
        applyConfig(ResetMode.kNoResetSafeParameters);
    }

    public void setBrakeMode(boolean value) {
        brakeMode = value;
        motorConfig.idleMode(value ? IdleMode.kBrake : IdleMode.kCoast);
        applyConfig(ResetMode.kNoResetSafeParameters);
    }

    public void setVelocityAccelerationError(int acceleration, int velocity, double error, ClosedLoopSlot slot) {
        motorConfig.closedLoop.maxMotion
                .maxAcceleration(acceleration, slot)
                .cruiseVelocity(velocity, slot)
                .allowedProfileError(error, slot);
        motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public double getSpeed() {
        return relEncoder.getVelocity();
    }

    public double getError() {
        return Math.abs(lastDesiredPosition - getPos());
    }

    public void setSpeed(double speed) {
        if (lastSpeed == speed)
            return;
        motor.set(speed);
        lastSpeed = speed;
    }

    public void setPos(double position) {
        logf("%s position:%.2f\n", name, position);
        lastDesiredPosition = position;
        lastSpeed = Double.NaN;
        motor.getClosedLoopController().setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void setVelocity(double velocity) {
        lastSpeed = Double.NaN;
        motor.getClosedLoopController().setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    public void setPosMotionMagic(double position) {
        lastDesiredPosition = position;
        lastSpeed = Double.NaN;
        motor.getClosedLoopController().setSetpoint(position, ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot2);
    }

    public void setVelocityMotionMagic(double velocity) {
        lastSpeed = Double.NaN;
        motor.getClosedLoopController().setSetpoint(velocity, ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot3);
    }

    public void stopMotor() {
        motor.stopMotor();
        lastSpeed = 0;
    }

    public double getMotorVoltage() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    public double getMotorCurrent() {
        return motor.getOutputCurrent();
    }

    public double getFollowCurrent() {
        if (followMotor == null) {
            return 0;
        }
        return followMotor.getOutputCurrent();
    }

    public double getPos() {
        return relEncoder.getPosition();
    }

    public void zeroEncoder() {
        relEncoder.setPosition(0);
    }

    public void setEncoderPosition(double position) {
        relEncoder.setPosition(position);
    }

    public boolean getForwardLimitSwitch() {
        return forwardSwitch.isPressed();
    }

    public boolean getReverseLimitSwitch() {
        return reverseSwitch.isPressed();
    }

    public void logPeriodic() {
        double pos = getPos();
        if (pos != lastPos) {
            lastPos = pos;
            if (myLogging) {
                logf("%s\n", getMotorVCS(motor, relEncoder));
                if (followMotor != null) {
                    logf("%s\n", getMotorVCS(followMotor, followEncoder));
                }
            }
        }
    }

    @Override
    public void periodic() {
        if (Robot.count % numberCyclesForDisplay == 0) {
            SmartDashboard.putNumber("Pos", getPos());
            SmartDashboard.putNumber("Cur", round2(motor.getOutputCurrent()));
            SmartDashboard.putNumber("Vel", round2(getSpeed()));
            SmartDashboard.putString("Mode", mode.toString());
            SmartDashboard.putNumber("RPM", round2(getSpeed() / velocityConversionFactor / 60));
            SmartDashboard.putNumber("Err", getError());
        }
        if (Robot.count % 10 == 0) {
            logPeriodic();
        }
        if (testMode) {
            testCases();
        }
    }

    void testTimes() {
        long startTime = RobotController.getFPGATime();
        double sp = motor.get();
        long endTime1 = RobotController.getFPGATime();
        double cur = motor.getOutputCurrent();
        long endTime2 = RobotController.getFPGATime();
        double enc = relEncoder.getPosition();
        long endTime3 = RobotController.getFPGATime();
        logf("%s motor sp:%.2f cur:%.2f pos:%.0f\n", name, sp, cur, enc);
        long endTime = RobotController.getFPGATime();
        System.out.printf("get:%d cur:%d enc:%d log:%d total:%d\n", endTime1 - startTime, endTime2 - endTime1,
                endTime3 - endTime2, endTime - endTime3, endTime - startTime);
    }

    private String getMotorVCS(SparkMax spark, RelativeEncoder encoder) {
        double bussVoltage = spark.getBusVoltage();
        double outputCurrent = spark.getOutputCurrent();
        return String.format("%s motor id:%d volts:%.2f cur:%.2f velocity:%.2f power:%.2f sp:%.3f", name,
                spark.getDeviceId(), bussVoltage, outputCurrent, encoder.getVelocity(),
                bussVoltage * outputCurrent, lastSpeed);
    }

    /** Returns the leader's measured velocity (was reading the absolute encoder, which is 0 on a bare NEO). */
    public double getLastSpeed() {
        return relEncoder.getVelocity();
    }

    public void logMotorVCS() {
        if (Math.abs(getLastSpeed()) > .01) {
            logf("%s\n", getMotorVCS(motor, relEncoder));
            if (followMotor != null)
                logf("%s\n", getMotorVCS(followMotor, followEncoder));
        }
    }

    public void setSmartTicks(int numberLoopsForDisplay) {
        if (numberLoopsForDisplay <= 0)
            this.numberCyclesForDisplay = Integer.MAX_VALUE;
        else
            this.numberCyclesForDisplay = numberLoopsForDisplay;
    }

    private void setVoltage(Voltage value) {
        // Clear the cached speed so a later setSpeed(0.0) is not skipped
        lastSpeed = Double.NaN;
        motor.setVoltage(value);
    }

    enum Modes {
        POSITION, VELOCITY, POSMOTIONMAGIC, VELMOTIONMAGIC, SPEED;

        public Modes next() {
            Modes[] values = Modes.values();
            int nextOrdinal = (this.ordinal() + 1) % values.length;
            return values[nextOrdinal];
        }
    }

    Modes mode = Modes.POSITION;
    boolean lastStart = false;
    double setPoint = 0;

    void testCases() {
        double value;
        // Hitting the start button moves to the next control method
        boolean start = controller.start().getAsBoolean();
        if (start && !lastStart) {
            stopMotor();
            setEncoderPosition(0.0);
            mode = mode.next();
            logf("***** Mode:%s for %s\n", mode, name);
        }
        lastStart = start;
        switch (mode) {
            case POSITION:
                value = controller.getHID().getPOV() / 10.0;
                if (value >= 0.0) {
                    if (setPoint != value)
                        logf("%s set position:%.6f\n", name, value);
                    setPos(value);
                    setPoint = value;
                }
                break;
            case VELOCITY:
                value = controller.getHID().getPOV() * 20.0;
                if (value >= 0.0) {
                    if (setPoint != value)
                        logf("%s set velocity:%.2f\n", name, value);
                    setVelocity(value);
                    setPoint = value;
                }
                break;
            case POSMOTIONMAGIC:
                value = controller.getHID().getPOV() / 10.0;
                if (value >= 0.0) {
                    if (setPoint != value)
                        logf("%s set position motion magic:%.6f\n", name, value);
                    setPosMotionMagic(value);
                    setPoint = value;
                }
                break;
            case VELMOTIONMAGIC:
                value = controller.getHID().getPOV() * 20.0;
                if (value >= 0.0) {
                    if (setPoint != value)
                        logf("%s set velocity motion magic:%.2f\n", name, value);
                    setVelocityMotionMagic(value);
                    setPoint = value;
                }
                break;
            case SPEED:
                value = Util.getSpeedFromTriggers(controller);
                if (Math.abs(value) > 0.15) {
                    if (setPoint != value)
                        logf("%s Set speed:%.2f\n", name, value);
                } else {
                    value = 0.0;
                }
                setSpeed(value);
                setPoint = value;
                break;
        }
        SmartDashboard.putNumber("SetP", setPoint);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> setSpeed(0.0))
                .withTimeout(0.5)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> setSpeed(0.0))
                .withTimeout(0.5)
                .andThen(sysId.dynamic(direction).withTimeout(1));
    }
}