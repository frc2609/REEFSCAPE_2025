package frc.robot.commands.pcmUtils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.PositionControlledMotor;

public class JogPCM extends Command {
    private double jogStep;
    private PositionControlledMotor pcm;
        
    public JogPCM(PositionControlledMotor pcm, double jogStep){
        this.pcm = pcm;
        this.jogStep = jogStep;
        addRequirements(pcm);
    }

    @Override
    public void execute() {
        double currentPosition = pcm.getPosition();
        pcm.goToPosition(currentPosition + jogStep);
    }

    @Override
    public void end(boolean interrupted) {
        pcm.stop();
    }

    @Override
    public boolean isFinished() {
        // Run continuously while the trigger is held.
        return false;
    }
}