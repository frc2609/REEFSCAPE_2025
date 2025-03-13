package frc.robot.commands.pcmUtils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.PositionControlledMotor;

public class MovePCM extends Command {
    private final PositionControlledMotor pcm;
    private final double position;

    public MovePCM(PositionControlledMotor pcm, double position){
        this.pcm = pcm;
        this.position = position;
        addRequirements(pcm);
    }

    @Override
    public void execute() {
        pcm.goToPosition(position);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
