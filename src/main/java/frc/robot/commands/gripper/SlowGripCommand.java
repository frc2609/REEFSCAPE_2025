package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

public class SlowGripCommand extends Command {
    private final Gripper gripper;
    public SlowGripCommand(Gripper gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void execute(){
        gripper.setSpeed(0.25);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
