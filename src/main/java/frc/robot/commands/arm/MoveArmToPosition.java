package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmToPosition extends Command {
     private final Arm arm;
     private final double position;

    public MoveArmToPosition(Arm arm, double position) {
        this.position = position;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.goToPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop arm when command ends
        arm.stop();
    }
}
