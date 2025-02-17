package frc.robot;
import java.util.Map;

public class AprilTagConfig {
    public static class AprilTag {
        public final int id;
        public final double x, y, z;
        public final double roll, pitch, yaw;

        public AprilTag(int id, double x, double y, double z, double roll, double pitch, double yaw) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.z = z;
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
        }
    }

    public static final AprilTag TAG_1 = new AprilTag(1, 1.0, 0.5, 0.3, 0.0, 0.0, 180.0);
    public static final AprilTag TAG_2 = new AprilTag(2, 2.0, 0.5, 0.3, 0.0, 0.0, 180.0);
    public static final AprilTag TAG_12 = new AprilTag(12, 3.0, 0.5, 0.3, 0.0, 0.0, 180.0);
    public static final AprilTag TAG_13 = new AprilTag(13, 4.0, 0.5, 0.3, 0.0, 0.0, 180.0);
    public static final AprilTag TAG_3 = new AprilTag(3, 5.0, 1.0, 0.5, 0.0, 0.0, 90.0);
    public static final AprilTag TAG_16 = new AprilTag(16, 6.0, 1.0, 0.5, 0.0, 0.0, 90.0);
    public static final AprilTag TAG_4 = new AprilTag(4, 7.0, 1.5, 0.7, 0.0, 0.0, 0.0);
    public static final AprilTag TAG_5 = new AprilTag(5, 8.0, 1.5, 0.7, 0.0, 0.0, 0.0);
    public static final AprilTag TAG_14 = new AprilTag(14, 9.0, 1.5, 0.7, 0.0, 0.0, 0.0);
    public static final AprilTag TAG_15 = new AprilTag(15, 10.0, 1.5, 0.7, 0.0, 0.0, 0.0);
    public static final AprilTag TAG_6 = new AprilTag(6, 11.0, 2.0, 0.9, 0.0, 0.0, -45.0);
    public static final AprilTag TAG_7 = new AprilTag(7, 12.0, 2.0, 0.9, 0.0, 0.0, -45.0);
    public static final AprilTag TAG_8 = new AprilTag(8, 13.0, 2.0, 0.9, 0.0, 0.0, -90.0);
    public static final AprilTag TAG_9 = new AprilTag(9, 14.0, 2.0, 0.9, 0.0, 0.0, -90.0);
    public static final AprilTag TAG_10 = new AprilTag(10, 15.0, 2.0, 0.9, 0.0, 0.0, -45.0);
    public static final AprilTag TAG_11 = new AprilTag(11, 16.0, 2.0, 0.9, 0.0, 0.0, -45.0);
    public static final AprilTag TAG_17 = new AprilTag(17, 17.0, 2.0, 0.9, 0.0, 0.0, -90.0);
    public static final AprilTag TAG_18 = new AprilTag(18, 18.0, 2.0, 0.9, 0.0, 0.0, -90.0);
    public static final AprilTag TAG_19 = new AprilTag(19, 19.0, 2.0, 0.9, 0.0, 0.0, -45.0);
    public static final AprilTag TAG_20 = new AprilTag(20, 20.0, 2.0, 0.9, 0.0, 0.0, -45.0);
    public static final AprilTag TAG_21 = new AprilTag(21, 21.0, 2.0, 0.9, 0.0, 0.0, -90.0);
    public static final AprilTag TAG_22 = new AprilTag(22, 22.0, 2.0, 0.9, 0.0, 0.0, -90.0);
}
