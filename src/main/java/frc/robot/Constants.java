package frc.robot;

import java.io.File;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public final class Constants {
    public static final class SwerveConstants {
        public static final File kSwerveConfigurationDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        public static final double kMaxSpeed = Units.feetToMeters(14);
        public static final boolean kMegaTag2Enabled = true;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriverControllerTranslationDeadband = 0.2;
        public static final double kDriverControllerRotationDeadband = 0.3;
        public static final int kOperatorControllerPort = 1;
    }
}
