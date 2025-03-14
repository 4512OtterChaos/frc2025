package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Derived from <a href="https://github.com/Spectrum3847/2025-Spectrum/blob/main/src/main/java/frc/spectrumLib/sim/RollerSim.java">Spectrum 3847</a>
 * 
 * <p> Draws a circle by spamming ligaments :)
 * 
 * <p> Optional roller stripe and colors
 */
public class MechanismCircle2d {

    private MechanismObject2d rollerAxle;
    private String name;

    private MechanismLigament2d[] circleBackground;
    private int backgroundLines;
    private Distance diameter;
    private Color8Bit color = new Color8Bit(Color.kBlack);

    private MechanismLigament2d rollerStripe;

    public MechanismCircle2d(
            MechanismObject2d axleRoot,
            int backgroundLines,
            Distance diameter,
            String name
            ) {
        this.backgroundLines = backgroundLines;
        this.diameter = diameter;
        this.name = name;
        this.rollerAxle = axleRoot;
        this.circleBackground = new MechanismLigament2d[this.backgroundLines];
        createBackground();
    }

    public MechanismCircle2d(
            MechanismObject2d axleRoot,
            int backgroundLines,
            Distance diameter,
            String name,
            Color8Bit color) {
        this(axleRoot, backgroundLines, diameter, name);
        this.color = color;
    }

    private void createBackground() {
        double radiusMeters = diameter.div(2).in(Meters);
        double diaInches = diameter.in(Inches);
        for (int i = 0; i < backgroundLines; i++) {
            circleBackground[i] =
                    rollerAxle.append(
                            new MechanismLigament2d(
                                    name + " Background " + i,
                                    radiusMeters,
                                    (360.0 / backgroundLines) * i,
                                    diaInches,
                                    color));
        }
    }

    public void setBackgroundAngle(Rotation2d angle) {
        for (int i = 0; i < backgroundLines; i++) {
            circleBackground[i].setAngle((360.0 / backgroundLines) * i + angle.getDegrees());
        }
    }

    public void createStripe() {
        createStripe(Color.kWhite, 5.0);
    }

    public void createStripe(Color stripeColor, double lineWidth) {
        rollerStripe =
                rollerAxle.append(
                        new MechanismLigament2d(
                                name + " Stripe",
                                diameter.div(2).in(Meters),
                                0.0,
                                diameter.in(Inches),
                                new Color8Bit(stripeColor)));
    }

    public void setStripeAngle(Rotation2d angle) {
        if (rollerStripe != null) {
            rollerStripe.setAngle(angle);
        }
    }

    public void setFullBackgroundColor(Color color) {
        for (int i = 0; i < backgroundLines; i++) {
            circleBackground[i].setColor(new Color8Bit(color));
        }
    }

    public void setStripedBackground(Color color1, Color color2) {
        for (int i = 0; i < backgroundLines; i++) {
            if (i % 2 == 0) {
                circleBackground[i].setColor(new Color8Bit(color1));
            } else {
                circleBackground[i].setColor(new Color8Bit(color2));
            }
        }
    }
}