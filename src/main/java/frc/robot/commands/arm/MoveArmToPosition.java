package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmToPosition extends Command {
    private final Arm arm;
    private final Supplier<Double> targetPositionSupplier;

    public MoveArmToPosition(Arm arm, Supplier<Double> targetPositionSupplier) {
        this.arm = arm;
        this.targetPositionSupplier = targetPositionSupplier;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        double currentTarget = targetPositionSupplier.get();
        arm.goToPosition(currentTarget);
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