package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;

/**
 * Elevator driven by a single Kraken X60 (TalonFX) through a 5:1 gearbox.
 *
 * - Bottom limit switch on a roboRIO DIO port zeroes the encoder and blocks
 * downward motion.
 * - Motor current limited to 20 A (stator and supply).
 * - moveToTop() / moveToBottom() use Motion Magic; bottom finishes by slowly
 * seeking the switch.
 *
 * Requires the CTRE Phoenix 6 vendor library.
 */
public class ElevatorSubsystem extends SubsystemBase {

  // ------------------------------------------------------------------
  // Constants — TUNE THESE FOR YOUR ROBOT
  // ------------------------------------------------------------------
  private static final class Constants {
    // Hardware
    static final int MOTOR_CAN_ID = 21;
    static final int BOTTOM_LIMIT_DIO = 1;
    /** Most limit switches wired to a DIO read TRUE when NOT pressed (pull-up). */
    static final boolean LIMIT_SWITCH_ACTIVE_LOW = true;
    static final boolean MOTOR_INVERTED = false; // positive output must raise the elevator

    // Mechanics
    static final double GEAR_RATIO = 5.0; // 5 motor rotations : 1 drum rotation
    static final double DRUM_DIAMETER_METERS = 0.0445; // sprocket/spool pitch diameter
    /**
     * Multiply by 2, 3... for continuous/cascade rigging where the carriage moves
     * faster than the chain.
     */
    static final double RIGGING_MULTIPLIER = 1.0;
    static final double METERS_PER_DRUM_ROTATION = Math.PI * DRUM_DIAMETER_METERS * RIGGING_MULTIPLIER;

    // Positions
    static final double TOP_HEIGHT_METERS = .4;
    static final double BOTTOM_HEIGHT_METERS = 0.0;
    static final double POSITION_TOLERANCE_METERS = 0.01;
    /**
     * When heading down and below this height, switch to slow seek for the limit
     * switch.
     */
    static final double BOTTOM_SEEK_START_METERS = 0.03;
    static final double SOFT_LIMIT_MARGIN_METERS = 0.02;

    // Current limits (amps)
    static final double CURRENT_LIMIT_AMPS = 20.0;

    // Homing
    static final double HOMING_VOLTS = -1.5; // slow, gentle drive downward

    // Closed-loop gains (Slot 0). Units are volts per drum rotation.
    static final double KP = 8.0;
    static final double KI = 0.0;
    static final double KD = 0.1;
    static final double KS = 0.15; // static friction
    static final double KV = 0.6; // volts per (drum rot/s) ≈ 12 V / free speed of drum
    static final double KA = 0.01;
    static final double KG = 0.35; // volts to hold the carriage against gravity

    // Motion Magic (drum rotations per second, per second^2)
    static final double CRUISE_VELOCITY = 15.0;
    static final double ACCELERATION = 40.0;
    static final double JERK = 400.0;
  }

  private enum Mode {
    IDLE, // motor neutral (brake)
    POSITION, // Motion Magic to targetRotations
    HOMING // driving slowly down until the limit switch is hit
  }

  // ------------------------------------------------------------------
  // Hardware and control requests
  // ------------------------------------------------------------------
  private final TalonFX motor = new TalonFX(Constants.MOTOR_CAN_ID);
  private final DigitalInput bottomLimit = new DigitalInput(Constants.BOTTOM_LIMIT_DIO);

  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut homingVoltage = new VoltageOut(0);
  private final NeutralOut neutral = new NeutralOut();

  // ------------------------------------------------------------------
  // State
  // ------------------------------------------------------------------
  private Mode mode = Mode.IDLE;
  private double targetRotations = 0.0; // drum (mechanism) rotations
  private boolean isHomed = false;
  private boolean wasAtBottom = false;

  public ElevatorSubsystem(CommandXboxController hid) {
    configureMotor();
    hid.povUp().onTrue(moveToTopCommand());
    hid.povDown().onTrue(moveToHeightCommand(.01));
    hid.start().onTrue(homeCommand());
    hid.back().onTrue(stopCommand());
    hid.povRight().whileTrue(bounceCommand());
    hid.povLeft().onTrue(moveToHeightCommand(.2));

    hid.y().onTrue(bounceCommand()); // start bouncing
    hid.a().onTrue(stopCommand()); // stop bouncing

  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Output direction and braking
    config.MotorOutput.Inverted = Constants.MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // 5:1 gearbox — all positions/velocities are now reported in DRUM rotations
    config.Feedback.SensorToMechanismRatio = Constants.GEAR_RATIO;

    // 20 A current limit
    config.CurrentLimits.StatorCurrentLimit = Constants.CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLowerLimit = Constants.CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Software upper limit as a backstop (bottom is handled by the limit switch)
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = metersToRotations(
        Constants.TOP_HEIGHT_METERS + Constants.SOFT_LIMIT_MARGIN_METERS);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // Gains, with gravity feedforward for an elevator
    config.Slot0.kP = Constants.KP;
    config.Slot0.kI = Constants.KI;
    config.Slot0.kD = Constants.KD;
    config.Slot0.kS = Constants.KS;
    config.Slot0.kV = Constants.KV;
    config.Slot0.kA = Constants.KA;
    config.Slot0.kG = Constants.KG;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // Motion profile
    config.MotionMagic.MotionMagicCruiseVelocity = Constants.CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = Constants.ACCELERATION;
    config.MotionMagic.MotionMagicJerk = Constants.JERK;

    // Retry config application a few times in case the CAN bus is busy at boot
    for (int i = 0; i < 5; i++) {
      if (motor.getConfigurator().apply(config).isOK()) {
        return;
      }
    }
    DriverStation.reportError("Elevator: failed to configure TalonFX", false);
  }

  // ------------------------------------------------------------------
  // Public API
  // ------------------------------------------------------------------

  /**
   * Move the elevator to the top position. Homes first if the encoder has never
   * been zeroed.
   */
  public void moveToTop() {
    if (!isHomed) {
      DriverStation.reportWarning("Elevator: not homed yet — homing before moving to top", false);
      home();
      return;
    }
    setTargetMeters(Constants.TOP_HEIGHT_METERS);
  }

  /**
   * Move the elevator to a position. Homes first if the encoder has never
   * been zeroed.
   */
  public void moveToHeight(double height) {
    if (!isHomed) {
      DriverStation.reportWarning("Elevator: not homed yet — homing before moving", false);
      home();
      return;
    }
    setTargetMeters(height);
  }

  /**
   * Move the elevator to the bottom. Finishes by slowly seeking the limit switch.
   */
  public void moveToBottom() {
    if (!isHomed) {
      home();
      return;
    }
    setTargetMeters(Constants.BOTTOM_HEIGHT_METERS);
  }

  /**
   * Drive slowly down until the bottom limit switch is pressed, then zero the
   * encoder.
   */
  public void home() {
    mode = Mode.HOMING;
  }

  /** Stop the motor (brake mode holds position as well as it can). */
  public void stop() {
    mode = Mode.IDLE;
  }

  /** True when the bottom limit switch is pressed. */
  public boolean isAtBottom() {
    boolean raw = bottomLimit.get();
    return Constants.LIMIT_SWITCH_ACTIVE_LOW ? !raw : raw;
  }

  public boolean isHomed() {
    return isHomed;
  }

  /** Current carriage height in meters. */
  public double getHeightMeters() {
    return rotationsToMeters(motor.getPosition().getValueAsDouble());
  }

  /** True once the current move has finished. */
  public boolean atTarget() {
    switch (mode) {
      case HOMING:
        return false;
      case POSITION:
        if (targetRotations <= metersToRotations(Constants.BOTTOM_HEIGHT_METERS)) {
          return isAtBottom();
        }
        return Math.abs(getHeightMeters() - rotationsToMeters(targetRotations)) < Constants.POSITION_TOLERANCE_METERS;
      case IDLE:
      default:
        return true;
    }
  }

  // ------------------------------------------------------------------
  // Command factories
  // ------------------------------------------------------------------

  /**
   * Move to a specific height (meters, clamped to the elevator's range).
   * Homes first if the encoder has never been zeroed, then moves, and ends
   * once the carriage is within tolerance of the target.
   */
  public Command moveToHeightCommand(double meters) {
    return Commands.sequence(
        homeCommand().unless(this::isHomed),
        runOnce(() -> setTargetMeters(meters)),
        Commands.waitUntil(this::atTarget).withTimeout(3.0),
        Commands.runOnce(() -> logf("Move to %.2f m done, at %.3f m, reached:%b\n",
            meters, getHeightMeters(), atTarget())))
        .withName("ElevatorToHeight(" + meters + ")");
  }

  public Command moveToTopCommand() {
    return runOnce(this::moveToTop)
        .andThen(Commands.waitUntil(() -> isHomed && atTarget() && mode == Mode.POSITION
            && targetRotations > 0))
        .andThen(Commands.print("Move to Top Complete"))
        .withName("ElevatorToTop");
  }

  public Command moveToBottomCommand() {
    return runOnce(this::moveToBottom)
        .andThen(Commands.waitUntil(this::isAtBottom))
        .andThen(Commands.print("Move to Bottom Complete"))
        .withName("ElevatorToBottom");
  }

  public Command homeCommand() {
    return runOnce(this::home)
        .andThen(Commands.waitUntil(this::isAtBottom))
        .andThen(Commands.print("Elevator Home Complete"))
        .withName("ElevatorHome");
  }

  public Command bounceCommand() {
    return Commands.sequence(
        moveToHeightCommand(.35),
        new WaitCommand(1),
        moveToHeightCommand(.1),
        new WaitCommand(1))
        .repeatedly()
        .finallyDo(this::stop)
        .withName("Bounce");
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("ElevatorStop");
  }

  // ------------------------------------------------------------------
  // Periodic loop — limit switch logic and motor output
  // ------------------------------------------------------------------
  @Override
  public void periodic() {
    boolean atBottom = isAtBottom();
    double motorCurrent = motor.getStatorCurrent().getValueAsDouble();
    if (motorCurrent > 500) {
      motor.set(0);
      logf("Current Limit Exceeded: %.2f\n", motorCurrent);
    }
    if (Robot.count % 50 == 0) {
      logf("Elevator limit:%b Mode:%s height:%.2f current:%.2f home:%b\n",
          atBottom, mode.name(), getHeightMeters(), motorCurrent, isHomed);
    }

    // Rising edge of the limit switch: zero the encoder
    if (atBottom && !wasAtBottom) {
      motor.setPosition(0.0);
      isHomed = true;
    }
    wasAtBottom = atBottom;

    // Disabled: reset to idle so the elevator doesn't jump on enable
    if (DriverStation.isDisabled()) {
      mode = Mode.IDLE;
    }

    switch (mode) {
      case HOMING:
        if (atBottom) {
          // Found the bottom — rest on the hard stop with no power
          mode = Mode.IDLE;
          motor.setControl(neutral);
        } else {
          motor.setControl(homingVoltage
              .withOutput(Constants.HOMING_VOLTS)
              .withLimitReverseMotion(atBottom));
        }
        break;

      case POSITION:
        boolean headingToBottom = targetRotations <= metersToRotations(Constants.BOTTOM_HEIGHT_METERS);
        if (headingToBottom && atBottom) {
          // Already resting on the switch — no need to push into it
          motor.setControl(neutral);
        } else if (headingToBottom
            && getHeightMeters() < Constants.BOTTOM_SEEK_START_METERS) {
          // Close to the bottom but switch not hit yet (encoder drift, chain stretch):
          // finish with a slow seek so we always land on the switch
          mode = Mode.HOMING;
        } else {
          // Limit switch blocks any further downward motion
          motor.setControl(motionMagic
              .withPosition(targetRotations)
              .withLimitReverseMotion(atBottom));
        }
        break;

      case IDLE:
      default:
        motor.setControl(neutral);
        break;
    }

    // Telemetry
    SmartDashboard.putNumber("Elevator/HeightMeters", getHeightMeters());
    SmartDashboard.putNumber("Elevator/TargetMeters", rotationsToMeters(targetRotations));
    SmartDashboard.putBoolean("Elevator/AtBottom", atBottom);
    SmartDashboard.putBoolean("Elevator/Homed", isHomed);
    SmartDashboard.putString("Elevator/Mode", mode.name());
    SmartDashboard.putNumber("Elevator/StatorCurrent", motorCurrent);
  }

  // ------------------------------------------------------------------
  // Helpers
  // ------------------------------------------------------------------
  private void setTargetMeters(double meters) {
    double clamped = Math.max(Constants.BOTTOM_HEIGHT_METERS,
        Math.min(Constants.TOP_HEIGHT_METERS, meters));
    targetRotations = metersToRotations(clamped);
    mode = Mode.POSITION;
  }

  private static double metersToRotations(double meters) {
    return meters / Constants.METERS_PER_DRUM_ROTATION;
  }

  private static double rotationsToMeters(double rotations) {
    return rotations * Constants.METERS_PER_DRUM_ROTATION;
  }
}

/*
 * Example usage in RobotContainer:
 *
 * private final ElevatorSubsystem elevator = new ElevatorSubsystem();
 * private final CommandXboxController driver = new CommandXboxController(0);
 *
 * private void configureBindings() {
 * driver.y().onTrue(elevator.moveToTopCommand());
 * driver.a().onTrue(elevator.moveToBottomCommand());
 * driver.x().onTrue(elevator.moveToHeightCommand(0.2));
 * driver.start().onTrue(elevator.homeCommand());
 * driver.back().onTrue(elevator.stopCommand());
 * }
 *
 * // Optional: home automatically at the start of teleop
 * // RobotModeTriggers.teleop().onTrue(elevator.homeCommand());
 */