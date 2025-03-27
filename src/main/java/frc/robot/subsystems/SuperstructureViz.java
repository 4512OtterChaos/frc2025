package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.util.MechanismCircle2d;

public class SuperstructureViz {

    private static final Distance kElevOffsetX = Inches.of(3.5);
    private static final Distance kElevOffsetZ = Inches.of(2.75);

    private static final Distance kCoralZ = Inches.of(24.25);
    private static final Distance kCoralLength = Inches.of(13.75);
    private static final Angle kCoralAngle = Degrees.of(-123);

    private static final Distance kCoralRollerX = Inches.of(11.1);
    private static final Distance kCoralRollerZ = Inches.of(15.8);

    private static final Distance kAlgaeRollerX = Inches.of(16.8);
    private static final Distance kAlgaeRollerZ = Inches.of(23.7);

    private static final double mechWidth = 1.0;
    private static final double mechHeight = 2.3;
    private Mechanism2d mech = new Mechanism2d(mechWidth, mechHeight);

    // Elevator
    private MechanismRoot2d elevRoot = mech.getRoot("elevatorRoot", mechWidth/2.0 + kElevOffsetX.in(Meters), kElevOffsetZ.in(Meters));
    private MechanismLigament2d mechElev = elevRoot.append(
            new MechanismLigament2d("elevator", 1, 90, 10, new Color8Bit(235, 137, 52)));
    // "Coral" / Carriage
    private MechanismLigament2d mechCoral = mechElev.append(
            new MechanismLigament2d("coral", kCoralLength.in(Meters), kCoralAngle.in(Degrees), 6, new Color8Bit(240, 240, 220)));

    private MechanismRoot2d coralRollerRoot = mech.getRoot("rollerRoot", 0, 0);
    private MechanismCircle2d coralRoller = new MechanismCircle2d(coralRollerRoot, 8, ManipulatorConstants.kCoralRollerDia, "roller");

    private MechanismRoot2d algaeRollerRoot = mech.getRoot("algaeRoot", 0, 0);
    private MechanismCircle2d algaeRoller = new MechanismCircle2d(algaeRollerRoot, 8, ManipulatorConstants.kAlgaeRollerDia, "algae");
    
    private MechanismRoot2d funnelRollerRoot = mech.getRoot("funnelRoot", mechWidth/2.0 - 0.177800, 0.813107); //TODO: Place root in the right position
    private MechanismCircle2d funnelRoller = new MechanismCircle2d(funnelRollerRoot, 8, ManipulatorConstants.kAlgaeRollerDia, "funnel");

    public SuperstructureViz() {
        coralRoller.setFullBackgroundColor(Color.kGreen);
        coralRoller.createStripe();

        algaeRoller.setFullBackgroundColor(Color.kOrange);
        algaeRoller.createStripe();
        
        funnelRoller.setFullBackgroundColor(Color.kRosyBrown);
        funnelRoller.createStripe();

        // coralRoller.setStripedBackground(Color.kGreen, Color.kWhite);
        // algaeRoller.setStripedBackground(Color.kOrange, Color.kWhite);
    }
    
    public void update(Distance elevHeight, Angle manipRollerAngle, Angle funnelRollerAngle) {
        var carriageCoralHeight = elevHeight.plus(kCoralZ.minus(kElevOffsetZ));
        mechElev.setLength(carriageCoralHeight.in(Meters));

        coralRollerRoot.setPosition(mechWidth/2.0 + kCoralRollerX.in(Meters), elevHeight.plus(kCoralRollerZ.minus(kElevOffsetZ)).in(Meters));
        Rotation2d coralRot = new Rotation2d(manipRollerAngle.unaryMinus());
        coralRoller.setBackgroundAngle(coralRot);
        coralRoller.setStripeAngle(coralRot);

        algaeRollerRoot.setPosition(mechWidth/2.0 + kAlgaeRollerX.in(Meters), elevHeight.plus(kAlgaeRollerZ.minus(kElevOffsetZ)).in(Meters));
        Rotation2d algaeRot = coralRot.unaryMinus();
        algaeRoller.setBackgroundAngle(algaeRot);
        algaeRoller.setStripeAngle(algaeRot);

        Rotation2d funnelRot = new Rotation2d(funnelRollerAngle);
        funnelRoller.setBackgroundAngle(funnelRot);
        funnelRoller.setStripeAngle(funnelRot);

        SmartDashboard.putData("Mech2d", mech);
    }
}
