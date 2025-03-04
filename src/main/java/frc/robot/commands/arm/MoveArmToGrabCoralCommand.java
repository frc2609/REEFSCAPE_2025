package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmToGrabCoralCommand extends Command {
    private final Arm arm;

    public MoveArmToGrabCoralCommand(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.goToPosition(-0.24);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop arm when command ends
        arm.stop();
    }
}