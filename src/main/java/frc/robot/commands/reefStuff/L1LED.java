package frc.robot.commands.reefStuff;

import frc.robot.subsystems.Led;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class L1LED extends Command {

  private final Led led;

  /**
   * Creates a new RainbowLed.
   *
   * @param RedLed The subsystem used by this command.
   */
  public L1LED(Led L1LED) {
    this.led = L1LED;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(L1LED);
  }

  @Override
  public void execute() {
    led.elevatorlvl1();
    led.Coral();
  }

  public boolean isFinished() {
    return false;
  }
}