package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmToPosition extends Command {
    private final Arm arm;
    private final double targetPosition;

    public MoveArmToPosition(Arm arm, double targetPosition) {
        this.arm = arm;
        this.targetPosition = targetPosition;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.goToPosition(targetPosition);
    }

    // @Override
    // public boolean isFinished() {
    //     return arm.atPosition();
    // }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
} 