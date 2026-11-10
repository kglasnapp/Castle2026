package frc.robot;

// import static frc.robot.Robot.yaw;
import static frc.robot.utilities.Util.logf;

import java.util.Optional;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.platforms.BlondeMini;
import frc.robot.platforms.DarrylMini;
import frc.robot.platforms.Mando;
import frc.robot.platforms.KeithMini;
import frc.robot.platforms.MiniMini;
import frc.robot.platforms.ParadeSrxDriveRobots;
import frc.robot.platforms.RobotRunnable;
import frc.robot.platforms.Sibling2025;
//import frc.robot.subsystems.DrivetrainJaguar;
import frc.robot.subsystems.DrivetrainSRX;
import frc.robot.subsystems.ElevatorSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since be
 * declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead,
 * the structure ofthe
 * robot (including subsystems, commands, and button mappings) should be
 * declared here.
 */
public class RobotContainer {
  private Optional<RobotRunnable> runnableRobot = Optional.empty();

  // public static LedSubsystem leds = new LedSubsystem();
  public DrivetrainSRX drivetrainSRX;

  // private MotorFlex motorFlex;
  // private MotorSparkMax motorSparkMax;
  // private MotorKraken motorKraken;
  // private MotorSRX motorSRX;

  // enum Motors {
  // FLEX, MAX, KRAKEN, SRX;

  // public Motors next() {
  // Motors[] values = Motors.values();
  // int nextOrdinal = (this.ordinal() + 1) % values.length;
  // return values[nextOrdinal];
  // }
  // }

  // private Motors motors = Motors.FLEX; // Set default motor for testing

  // private void setMotorForTest() {
  // testFlex = false;
  // testSmartMax = false;
  // testKraken = false;
  // testSRX = false;
  // motors = motors.next(); // Get the next mode
  // logf("************** Motor:%s\n", motors.toString());
  // switch (motors) {
  // case FLEX:
  // testFlex = true;
  // break;
  // case MAX:
  // testSmartMax = true;
  // break;
  // case KRAKEN:
  // testKraken = true;
  // break;
  // case SRX:
  // testSRX = true;
  // break;
  // }
  // motorFlex.setTestMode(testFlex);
  // motorFlex.setLogging(testFlex);
  // motorFlex.setSmartTicks(testFlex ? 2 : 0);
  // motorSparkMax.setTestMode(testSmartMax);
  // motorSparkMax.setLogging(testSmartMax);
  // motorSparkMax.setSmartTicks(testSmartMax ? 2 : 0);
  // motorKraken.setTestMode(testKraken);
  // motorKraken.setLogging(testKraken);
  // motorKraken.setSmartTicks(testKraken ? 1 : 0);
  // motorSRX.setTestMode(testSRX);
  // motorSRX.setLogging(testSRX);
  // motorSRX.setSmartTicks(testSRX ? 2 : 0);
  // SmartDashboard.putString("Motor", motors.toString());
  // }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CommandXboxController driveController = new CommandXboxController(0);
    XboxController driveHID = driveController.getHID();

    // double kP;
    // double kI;
    // double kD;
    // PIDController pidNeo = new PIDController(kP, kI, kD);

    // Set the default Robot Mode to Cube
    switch (Config.robotType) {
      case Simulation:
        driveController.back().whileTrue(zeroYawCommand);
        break;
      case BlondeMini:
        runnableRobot = Optional.of(new BlondeMini(driveController));
        break;
      case DarrylMini:
        runnableRobot = Optional.of(new DarrylMini());
        break;
      case MiniMini:
        runnableRobot = Optional.of(new MiniMini(driveController));
        break;
      case KeithMini: // Test mini
        runnableRobot = Optional.of(new KeithMini(driveController));
        break;
      case SurfBoard:
        Robot.config.driveRightFollow = 4;
        Robot.config.driveLeftFollow = 5;
        runnableRobot = Optional.of(new ParadeSrxDriveRobots(driveHID, "SurfBoard"));
        new ElevatorSubsystem( driveController);

        break;
      case Kevin: // Ginger Bread Robot
        runnableRobot = Optional.of(new ParadeSrxDriveRobots(driveHID, "Kevin"));
        break;
      case Wooly: // Big ball shooter
        // Uses Jaguars for drive train and shooter
        // Uses PCM to control shooter tilt and shooter activate
        Robot.config.driveTrainJaguar = true;
        Robot.config.enableCompressor = true;
        Robot.config.pneumaticType = PneumaticsModuleType.CTREPCM;
        // TODO Will need a rewrite to support wolly using Jaguars
        // for drive train and shooter. Need to add a Jaguar subsystem for shooter.
        runnableRobot = Optional.of(new Mando(driveHID));
        break;
      case Sibling2025:
        runnableRobot = Optional.of(new Sibling2025(driveHID));
        break;
      case Mando:
        runnableRobot = Optional.of(new Mando(driveHID));
        break;
      case MiniIsaac:
        break;

    }
    logf("Finished Creating RobotContainer\n");
  }

  public Optional<RobotRunnable> robot() {
    return runnableRobot;
  }

  Command zeroYawCommand = new InstantCommand(
      new Runnable() {
        public void run() {
          Robot.yawProvider.zeroYaw();
        }
      });
}
