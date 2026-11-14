// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DrivetrainSRX extends SubsystemBase {
  // ---- Tuning constants ----
  /** Stick values smaller than this are treated as zero (stops motor creep). */
  private static final double STICK_DEADBAND = 0.04;
  /** Max change in motor output per second (1/rate = seconds from 0 to full). 3.0 => ~0.33 s. */
  private static final double SLEW_RATE_PER_SEC = 3.0;
  /** Proportional gain for drive-straight heading correction. */
  private static final double DRIVE_STRAIGHT_KP = 0.035; // Was 0.045
  /** Heading error (degrees) is clamped to +/- this value. */
  private static final double MAX_HEADING_ERROR_DEG = 10.0;
  /**
   * Drive mode at startup. false = tank (left stick Y = left side, right stick Y = right side).
   * true = arcade (left stick Y = forward/back, right stick X = turn).
   */
  private static final boolean DEFAULT_ARCADE_MODE = true;
  /** Scales the turn stick in arcade mode (1.0 = full turn rate; lower for gentler turning). */
  private static final double ARCADE_TURN_SCALE = 0.3;
  private static final double ARCADE_DRIVE_SCALE = 0.5;

  // ---- Hardware ----
  private final TalonSRX talonDriveRight = new TalonSRX(Robot.config.driveRight);
  private final TalonSRX talonDriveRightFollow = new TalonSRX(Robot.config.driveRightFollow);
  private final TalonSRX talonDriveLeft = new TalonSRX(Robot.config.driveLeft);
  private final TalonSRX talonDriveLeftFollow = new TalonSRX(Robot.config.driveLeftFollow);
  private final XboxController driveController;

  // Per-instance limiters (not static / shared). Tank uses left/right, arcade uses forward/turn.
  private final SlewRateLimiter leftLimiter = new SlewRateLimiter(SLEW_RATE_PER_SEC);
  private final SlewRateLimiter rightLimiter = new SlewRateLimiter(SLEW_RATE_PER_SEC);
  private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(SLEW_RATE_PER_SEC);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(SLEW_RATE_PER_SEC);

  // ---- State ----
  /** Final left/right motor commands for this loop (after tank/arcade mixing). */
  public double leftStick;
  public double rightStick;
  private Double targetAngle = null; // null => drive-straight not active
  private boolean arcadeMode = DEFAULT_ARCADE_MODE;

  public DrivetrainSRX(XboxController driveController) {
    logf("Start of Drive Train for SRX Subsystem\n");
    this.driveController = driveController;

    // Reset ALL controllers first, then configure following/inversion.
    // (configFactoryDefault after follow() can clear settings, and followers
    // were never being reset at all.)
    talonDriveRight.configFactoryDefault();
    talonDriveRightFollow.configFactoryDefault();
    talonDriveLeft.configFactoryDefault();
    talonDriveLeftFollow.configFactoryDefault();

    talonDriveRightFollow.follow(talonDriveRight);
    talonDriveLeftFollow.follow(talonDriveLeft);

    // The right side is mirrored, so invert it in the motor controller instead of
    // flipping the joystick sign. This keeps "positive = forward" for BOTH sides,
    // which drive-straight depends on.
    // !! Verify on blocks: both sides should spin forward (green LEDs) with sticks pushed forward.
    talonDriveLeft.setInverted(true);
    talonDriveRight.setInverted(false);
    talonDriveLeftFollow.setInverted(InvertType.FollowMaster);
    talonDriveRightFollow.setInverted(InvertType.FollowMaster);

    talonDriveLeft.setNeutralMode(NeutralMode.Brake);
    talonDriveRight.setNeutralMode(NeutralMode.Brake);
    talonDriveLeftFollow.setNeutralMode(NeutralMode.Brake);
    talonDriveRightFollow.setNeutralMode(NeutralMode.Brake);
  }

  /** Switch between arcade (true) and tank (false) drive. Safe to call at any time. */
  public void setArcadeMode(boolean arcade) {
    if (arcade != arcadeMode) {
      // Reset the limiters so a stale value from the other mode can't cause a jump.
      leftLimiter.reset(0);
      rightLimiter.reset(0);
      forwardLimiter.reset(0);
      turnLimiter.reset(0);
      arcadeMode = arcade;
      logf("Drive mode changed to %s\n", arcade ? "ARCADE" : "TANK");
    }
  }

  public boolean isArcadeMode() {
    return arcadeMode;
  }

  @Override
  public void periodic() {
    // Xbox Y axes are negative when pushed forward, so negate them to make forward positive.
    if (arcadeMode) {
      double forward = forwardLimiter.calculate(
          MathUtil.applyDeadband(driveController.getLeftY(), STICK_DEADBAND)) * ARCADE_DRIVE_SCALE;
      double turn = turnLimiter.calculate(
          MathUtil.applyDeadband(driveController.getRightX(), STICK_DEADBAND)) * ARCADE_TURN_SCALE;
      // Right stick pushed right => positive turn => left side faster => robot turns right.
      leftStick = forward + turn;
      rightStick = forward - turn;
      // Desaturate: if either side exceeds 1, scale both down to keep the turn ratio.
      double max = Math.max(Math.abs(leftStick), Math.abs(rightStick));
      if (max > 1.0) {
        leftStick /= max;
        rightStick /= max;
      }
    } else {
      // Tank drive. Use the slew-rate limiter outputs (they were previously discarded).
      leftStick = leftLimiter.calculate(
          MathUtil.applyDeadband(driveController.getLeftY(), STICK_DEADBAND));
      rightStick = rightLimiter.calculate(
          MathUtil.applyDeadband(driveController.getRightY(), STICK_DEADBAND));
    }

    if (Robot.count % 250 == 5) { // set to -1 to disable this log
      logf("Drive %s left:%.2f right:%.2f yaw:%.2f target:%s\n", arcadeMode ? "arcade" : "tank",
          leftStick, rightStick, Robot.yaw, formatTarget());
    }
    SmartDashboard.putNumber("Right Stick", rightStick);
    SmartDashboard.putNumber("Left Stick", leftStick);
    SmartDashboard.putBoolean("Drive Straight", targetAngle != null);
    SmartDashboard.putBoolean("Arcade Mode", arcadeMode);

    if (driveController.getLeftBumperButtonPressed()) {
      targetAngle = Robot.yaw;
      logf("Drive Straight start yaw:%.2f\n", Robot.yaw);
    }
    if (driveController.getLeftBumperButtonReleased()) {
      logf("Drive Straight finish goal:%s yaw:%.2f\n", formatTarget(), Robot.yaw);
      targetAngle = null;
    }
    if (targetAngle != null) {
      driveStraight();
    }

    talonDriveLeft.set(ControlMode.PercentOutput, MathUtil.clamp(leftStick, -1.0, 1.0));
    talonDriveRight.set(ControlMode.PercentOutput, MathUtil.clamp(rightStick, -1.0, 1.0));
  }

  private void driveStraight() {
    // Wrap into [-180, 180) so crossing the +/-180 boundary doesn't produce a 360 deg error.
    double rawError = MathUtil.inputModulus(Robot.yaw - targetAngle, -180.0, 180.0);
    double error = MathUtil.clamp(rawError, -MAX_HEADING_ERROR_DEG, MAX_HEADING_ERROR_DEG);

    if (error != rawError && Robot.count % 50 == 7) { // rate-limited, was logging every loop
      logf("!!!!! Drive Straight error clamped diff:%.1f yaw:%.1f target:%.3f\n",
          rawError, Robot.yaw, targetAngle);
    }

    // In arcade mode this average equals the forward stick, so drive-straight
    // holds heading and overrides the turn stick while the bumper is held.
    double averageJoy = (rightStick + leftStick) / 2.0; // was "/ 1.0"
    // Correction scales with speed so it doesn't spin the robot when nearly stopped.
    // Sign assumes yaw increases clockwise (NavX convention). If your gyro is CCW-positive
    // (Pigeon / WPILib convention) the robot will veer away from target: negate the factor.
    double factor = error * Math.abs(averageJoy) * DRIVE_STRAIGHT_KP;

    if (Robot.count % 50 == 6) { // log about once per second
      logf("Drive Straight targ:%.2f yaw:%.2f err:%.2f avg:%.2f factor:%.2f Joy:<%.2f,%.2f>\n",
          targetAngle, Robot.yaw, error, averageJoy, factor, leftStick, rightStick);
    }
    leftStick = averageJoy - factor;
    rightStick = averageJoy + factor;
  }

  private String formatTarget() {
    return targetAngle == null ? "none" : String.format("%.2f", targetAngle);
  }

/*
To switch modes from the controller, you could add one line in RobotContainer, 
for example:new JoystickButton(driveController, XboxController.Button.kStart.value).onTrue(
Commands.runOnce(() -> drivetrain.setArcadeMode(!drivetrain.isArcadeMode())));
*/

}
