package frc.robot.utils;

import edu.wpi.first.wpilibj.Encoder;

public class EncoderAdapter implements IEncoder{
    private Encoder encoder;

    public EncoderAdapter(Encoder encoder) {
        this.encoder = encoder;
    }

    @Override
    public double getPosition() {
        return encoder.getDistance();
    }
}
