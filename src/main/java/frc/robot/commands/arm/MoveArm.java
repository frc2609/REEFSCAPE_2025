package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;

public class MoveArm extends SequentialCommandGroup {
    private static final double UP_POSITION = -0.2;
    private static final double DOWN_POSITION = 0.2;
    private static final double HOME_POSITION = 0;

    public MoveArm(Arm arm) {
        addCommands(
            new MoveArmToPosition(arm, UP_POSITION).withTimeout(1),
            new WaitCommand(0.2),
            new MoveArmToPosition(arm, DOWN_POSITION).withTimeout(1),
            new WaitCommand(0.2),
            new MoveArmToPosition(arm, HOME_POSITION).withTimeout(1)
        );
    }
}
