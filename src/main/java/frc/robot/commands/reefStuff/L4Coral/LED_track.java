package frc.robot.commands.reefStuff.L4Coral;

import frc.robot.subsystems.Led;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LED_track extends Command {

    private final Led led;
    /**
     * Creates a new RainbowLed.
     *
     * @param LED_track The subsystem used by this command.
     * @param coral
     */
    public LED_track(Led LED_track) {
        this.led = LED_track;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(LED_track);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    @Override
    public void execute() {
        led.track_PID();
    }

    public boolean isFinished() {
        return false;
    }
}