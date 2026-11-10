package frc.robot.utilities;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class myCommands extends SubsystemBase {
    long myCount = 0;

    // public static Command myLogf(String pattern, Object... args) {
    //     return runOnce(() -> logf(pattern, args));
    //     //return null;
    // }

    public Command myPrint(String s) {
        return new InstantCommand(() -> logf(s));
    }

    public Command IncrementWithWaitChain() {
        return new InstantCommand(() -> myCount += 1)
                .andThen(new InstantCommand(() -> logf("1st count:%d", myCount)))
                .andThen(new InstantCommand(() -> myCount += 1))
                .andThen(new WaitCommand(1))
                .andThen(new InstantCommand(() -> logf("2nd count:%d", myCount)))
                .andThen(new WaitCommand(1))
                .andThen(new InstantCommand(() -> logf("3rd count:%d", myCount)))
                .andThen(new WaitCommand(1))
                .andThen(new InstantCommand(() -> logf("4th count:%d", myCount)));
    }

    public Command showCount() {
        return runOnce(() -> logf("myCount:%d", myCount));
    }

}