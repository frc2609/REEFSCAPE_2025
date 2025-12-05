package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

public class GripCommand extends Command {
    private final Gripper gripper;
    public GripCommand(Gripper gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void execute(){
        gripper.setSpeed(0.75);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
