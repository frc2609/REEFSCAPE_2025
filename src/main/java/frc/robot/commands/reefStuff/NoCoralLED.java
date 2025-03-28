package frc.robot.commands.reefStuff;

import frc.robot.subsystems.Led;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class NoCoralLED extends Command {

    private final Led led;

    /**
     * Creates a new RainbowLed.
     *
     * @param NoCoralLed The subsystem used by this command.
     */
    public NoCoralLED(Led NoCoralLED) {
        this.led = NoCoralLED;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(NoCoralLED); 
    }

    @Override
    public void execute() {
        led.Coral();
    }

    public boolean isFinished() {
        return true;
    }
}
    

