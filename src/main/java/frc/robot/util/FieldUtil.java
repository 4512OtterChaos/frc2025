package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;

public class FieldUtil {
    public static final Distance kFieldWidth = Centimeters.of(805);
    public static final Distance kFieldLength = Centimeters.of(1755);

    public static final Translation2d kReefTrl = new Translation2d(
        Units.feetToMeters(12),
        kFieldWidth.div(2).in(Meters)
    );

    public static final Distance kReefWidth = Centimeters.of(166);
    public static final Distance kReefPoleDist = Centimeters.of(33);
    
}
