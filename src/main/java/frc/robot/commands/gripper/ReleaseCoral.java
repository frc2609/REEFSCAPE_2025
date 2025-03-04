package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

public class ReleaseCoral extends Command {
    private final Gripper gripper;
    public ReleaseCoral(Gripper gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void execute(){
        gripper.setCoralSpeed(1);
    }

    @Override
    public void end(boolean interrupted){
        gripper.stopCoral();
    }
}
