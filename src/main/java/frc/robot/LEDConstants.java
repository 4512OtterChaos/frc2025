package frc.robot;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
    public static final int kPort = 9;
    public static final int kLength = 15;

    public static final Color kColorRiptideBlue = new Color(0, 101, 253);
    public static final Color kColorBlueShade2 = new Color(212, 228, 253);
    public static final Color kColorChaosOrange = new Color(253, 101, 0);

    public static final LEDPattern kPatternBlueGradient = LEDPattern.gradient(GradientType.kContinuous, kColorRiptideBlue, kColorBlueShade2);
    public static final LEDPattern kPatternBlueScroll = kPatternBlueGradient.scrollAtRelativeSpeed(Percent.of(67).per(Second));
    
    public static final LEDPattern kPatternGreenSolid = LEDPattern.solid(Color.kGreen);
    public static final LEDPattern kPatternGreenBlink = kPatternGreenSolid.blink(Seconds.of(0.1), Seconds.of(0.1));

    public static final LEDPattern kPatternOrangeSolid = LEDPattern.solid(kColorChaosOrange);
    public static final LEDPattern kPatternOrangeBlink = kPatternOrangeSolid.blink(Seconds.of(0.15), Seconds.of(0.15));

    public static final LEDPattern kPatternPurpleSolid = LEDPattern.solid(Color.kPurple);
    public static final LEDPattern kPatternPurpleBlink = kPatternPurpleSolid.blink(Seconds.of(0.1), Seconds.of(0.1));
}
