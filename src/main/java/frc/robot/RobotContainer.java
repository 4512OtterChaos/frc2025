// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.traits.CommonTalon;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import static frc.robot.subsystems.drivetrain.DriveConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoRoutines;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FieldUtil;
import frc.robot.util.OCXboxController;
import frc.robot.util.FieldUtil.Alignment;
import frc.robot.util.FieldUtil.ReefFace;

public class RobotContainer {
        
    private final Telemetry logger = new Telemetry(kMaxLinearSpeed);
    
    public final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
    public final Manipulator manipulator = new Manipulator();
    public final Funnel funnel = new Funnel();
    public final Elevator elevator = new Elevator();
    
    private OCXboxController driver = new OCXboxController(0);
    private OCXboxController operator = new OCXboxController(1);

    public final Superstructure superstructure = new Superstructure(swerve, manipulator, funnel, elevator);
    
    private final Vision vision = new Vision();

    private final Trigger nearCoralStation = new Trigger(() -> {
        var swervePose = swerve.getGlobalPoseEstimate();
        boolean nearStation = FieldUtil.nearestCoralStation(swervePose).relativeTo(swervePose).getTranslation().getNorm() < 1.6;
        boolean hasFMS = DriverStation.isFMSAttached() || true; //TODO: remove || true
        return nearStation && (hasFMS || Robot.isSimulation());
    });

    private AddressableLED led = new AddressableLED(9);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(15);

    private LEDPattern layer1Pattern = LEDConstants.kPatternBlueScroll;
    private LEDPattern layer2Pattern = LEDPattern.kOff;
    private LEDPattern layer3Pattern = LEDPattern.kOff;
    private LEDPattern layer4Pattern = LEDPattern.kOff;
    
    /* Path follower */
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    private enum ResetBehavior {
        FULL_RESET,
        ROT_RESET,
        NO_RESET
    }
    private final SendableChooser<ResetBehavior> resetBehaviorChooser = new SendableChooser<>();
    
    public RobotContainer() {
        autoRoutines = new AutoRoutines(superstructure, swerve, elevator, manipulator);
        
        autoChooser.addCmd("TaxiAuto", autoRoutines::taxiAuto);
        autoChooser.addCmd("TaxiFarAuto", autoRoutines::taxiFar);
        autoChooser.addCmd("LeftWall 3CoralL4", () -> autoRoutines.Wall3CoralL4(false));
        autoChooser.addCmd("RightWall 3CoralL4", () -> autoRoutines.Wall3CoralL4(true));
        autoChooser.addCmd("Middle 1CoralL4", () -> autoRoutines.Middle1CoralL4());
        autoChooser.addCmd("Middle 1CoralL4 1Algae", () -> autoRoutines.Middle1CoralL41Algae());
        autoChooser.addCmd("Middle 1CoralL4 2Algae", () -> autoRoutines.Middle1CoralL42Algae());
        // autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        resetBehaviorChooser.addOption("* FULL RESET *", ResetBehavior.FULL_RESET);
        resetBehaviorChooser.addOption("ROTATION RESET", ResetBehavior.ROT_RESET);
        resetBehaviorChooser.addOption("NO RESET", ResetBehavior.NO_RESET);
        SmartDashboard.putData("Reset Behavior", resetBehaviorChooser);
        
        configureDefaultBindings();
        configureDriverBindings(driver);
        // configureOperatorBindings(driver);
        configureOperatorBindings(operator);
        if (Robot.isSimulation() || true){//TODO: Delete when in pits
            simBindings(driver);
        }
        // configureOperatorBindings(operator);
        TalonFX[] swerveMotors = {
            swerve.getModule(0).getDriveMotor(),
            swerve.getModule(0).getSteerMotor(),
            swerve.getModule(1).getDriveMotor(),
            swerve.getModule(1).getSteerMotor(),
            swerve.getModule(2).getDriveMotor(),
            swerve.getModule(2).getSteerMotor(),
            swerve.getModule(3).getDriveMotor(),
            swerve.getModule(3).getSteerMotor()
        };
        setSwerveUpdateFrequency(swerveMotors);
        ParentDevice.optimizeBusUtilizationForAll(swerveMotors);

        SmartDashboard.putData("Commands/ResetHeading0", runOnce(()->swerve.resetRotation(Rotation2d.kZero)).withName("Reset heading 0"));
        SmartDashboard.putData("Commands/ResetHeading180", runOnce(()->swerve.resetRotation(Rotation2d.k180deg)).withName("Reset heading 180"));
        
        bindLEDAnimations();
        led.setLength(15);
        led.start();

        DataLogManager.start();
    }

    public void periodic() {
        superstructure.periodic();
        vision.periodic();

        var resetBehavior = resetBehaviorChooser.getSelected();
        if (resetBehavior != null) {
            switch(resetBehavior){
                case FULL_RESET -> autoRoutines.setShouldResetPose(true, true);
                case ROT_RESET -> autoRoutines.setShouldResetPose(false, true);
                case NO_RESET -> autoRoutines.setShouldResetPose(false, false);
            };
        };

        // update pose estimator with vision measurements
        double phoenixTimeOffset = Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds();
        var swerveState = swerve.getState();
        vision.update(
            swerve.visionEstimator,
            swerveState.Pose.getRotation(),
            RadiansPerSecond.of(swerveState.Speeds.omegaRadiansPerSecond),
            swerveState.Timestamp + phoenixTimeOffset
        );

        // LED patterns
        if (!layer4Pattern.equals(LEDPattern.kOff)) {
            layer4Pattern.applyTo(ledBuffer);
        }
        else if (!layer3Pattern.equals(LEDPattern.kOff)) {
            layer3Pattern.applyTo(ledBuffer);
        }
        else if (!layer2Pattern.equals(LEDPattern.kOff)) {
            layer2Pattern.applyTo(ledBuffer);
        }
        else {
            layer1Pattern.applyTo(ledBuffer);
        }
        led.setData(ledBuffer);
    }
    
    public void setSwerveUpdateFrequency(CommonTalon... motors) {
        for (var motor : motors) {
            motor.getDutyCycle().setUpdateFrequency(100);
            motor.getMotorVoltage().setUpdateFrequency(100);
            motor.getPosition().setUpdateFrequency(100);
            motor.getVelocity().setUpdateFrequency(50);
            motor.getStatorCurrent().setUpdateFrequency(50);
        }
    }

    public void configureDefaultBindings() {
        manipulator.setDefaultCommand(either(
            manipulator.holdAlgaeC(),
            manipulator.holdPositionC(),
            manipulator.hasAlgae
        ));

        funnel.setDefaultCommand(funnel.slowFeedCoralC());
        // Automatically feed coral to a consistent position when detected
        manipulator.isCoralDetected().and(()->manipulator.getCurrentCommand() != null && manipulator.getCurrentCommand().equals(manipulator.getDefaultCommand()))
            .onTrue(superstructure.feedCoralSequenceC());

        // Automatically start intaking if close to station
        nearCoralStation.and(manipulator.hasCoral.negate()).whileTrue(
            sequence(
                superstructure.feedCoralFastSequenceC().deadlineFor(
                    funnel.feedCoralC()
                ).until(manipulator.isCoralDetected()),
                superstructure.feedCoralSequenceC().asProxy()
            )
        );
    }

    private void configureDriverBindings(OCXboxController controller) {
        //DRIVE COMMAND
        Supplier<ChassisSpeeds> driveSupplier = () -> controller.getSpeeds(
            swerve.driveSpeed.in(MetersPerSecond),
            swerve.turnSpeed.in(RadiansPerSecond));
        swerve.setDefaultCommand(swerve.drive(driveSupplier).withName("D:Controller Drive"));

        Trigger driverSomeLeftInput = new Trigger(() -> {
            return Math.hypot(controller.getLeftX(), controller.getLeftY()) > OCXboxController.kDeadband;
        });
        Trigger driverSomeRightInput = new Trigger(() -> {
            return Math.hypot(controller.getRightX(), controller.getRightY()) > OCXboxController.kDeadband;
        });
        Trigger driverMaxLeftInput = new Trigger(() -> {
            return Math.hypot(controller.getLeftX(), controller.getLeftY()) > 0.9;
        });
        // snap to coral station angle
        nearCoralStation
            .and(()->swerve.getCurrentCommand() != null && swerve.getCurrentCommand().equals(swerve.getDefaultCommand()))
            .and(driverSomeRightInput.negate().debounce(0.5))
            .and(()->DriverStation.isTeleopEnabled())
            .onTrue(
                swerve.driveFacingAngle(
                    driveSupplier,
                    () -> FieldUtil.nearestCoralStation(swerve.getGlobalPoseEstimate()).getRotation(),
                    true, false
                ).until(driverSomeRightInput.or(nearCoralStation.negate())).withName("FacingCoralStation")
            );
        
        // snap to reef angle
        nearCoralStation.negate().and(manipulator.hasAlgae.negate())
            .and(()->swerve.getCurrentCommand() != null && swerve.getCurrentCommand().equals(swerve.getDefaultCommand()))
            .and(driverSomeRightInput.negate().debounce(0.5))
            .and(()->DriverStation.isTeleopEnabled())
            .onTrue(
                swerve.driveFacingAngle(
                    driveSupplier,
                    () -> ReefFace.getClosest(swerve.getGlobalPoseEstimate(), Alignment.CENTER).getAlignmentPose(Alignment.CENTER).getRotation(),
                    true, false
                ).until(driverSomeRightInput.or(nearCoralStation))
            );
        
        // auto-stow
        new Trigger(() -> {
            return Math.hypot(controller.getLeftX(), controller.getLeftY()) > 0.75;
        }).debounce(0.15).and(swerve.isAligning.negate()).and(()->DriverStation.isTeleop()).onTrue(elevator.setMinC());
        
        // reset the robot heading to forward
        controller.start().onTrue(swerve.runOnce(() -> swerve.resetRotation(Rotation2d.kZero)));

        controller.leftTrigger().and(controller.rightTrigger().negate()).whileTrue(swerve.alignToReef(Alignment.LEFT, true));
        controller.rightTrigger().and(controller.leftTrigger().negate()).whileTrue(swerve.alignToReef(Alignment.RIGHT, true));
        controller.leftTrigger().and(controller.rightTrigger()).whileTrue(swerve.alignToReef(Alignment.CENTER, true));

        controller.leftBumper().whileTrue(manipulator.scoreAlgaeC());
        controller.rightBumper().whileTrue(manipulator.scoreCoralC());

        controller.povRight().whileTrue(funnel.feedCoralC());

        controller.a().whileTrue(superstructure.autoAlgaePickUp());
        controller.b().whileTrue(superstructure.algaeShoot());
        
        // controller.leftTrigger().whileTrue(swerve.drive(() -> new ChassisSpeeds(0, controller.getLeftTriggerAxis() * 0.3, 0), false, true).withName("Strafe Left"));
        // controller.rightTrigger().whileTrue(swerve.drive(() -> new ChassisSpeeds(0, controller.getRightTriggerAxis() * -0.3, 0), false, true).withName("Strafe Right"));       
        
        swerve.registerTelemetry(logger::telemeterize);
    }
    
    private void configureOperatorBindings(OCXboxController controller) {
        //===== ELEVATOR COMMANDS
        controller.povUp().onTrue(elevator.setMinC());
        controller.a().onTrue(elevator.setL1C()
            .beforeStarting(waitUntil(manipulator.isCoralDetected().negate())));
        controller.x().and(controller.rightBumper().negate()).onTrue(elevator.setL2C()
            .beforeStarting(waitUntil(manipulator.isCoralDetected().negate())));
        controller.y().onTrue(elevator.setL3C()
            .beforeStarting(waitUntil(manipulator.isCoralDetected().negate())));
        controller.b().onTrue(elevator.setL4C()
            .beforeStarting(waitUntil(manipulator.isCoralDetected().negate())));

        controller.rightBumper().onTrue(elevator.setAlgaeL3C()
            .beforeStarting(waitUntil(manipulator.isCoralDetected().negate())));
        //=====
    }

    private void simBindings(OCXboxController controller) {
        controller.povLeft().onTrue(runOnce(()->swerve.disturbGlobalPoseEstimate()));
        
    }

    private void bindLEDAnimations() {
        manipulator.isCoralDetected().whileTrue(runEnd(
            ()->layer2Pattern = LEDConstants.kPatternGreenSolid,
            ()->layer2Pattern = LEDPattern.kOff
        ));

        swerve.isAligning.whileTrue(runEnd(
            ()->layer3Pattern = LEDConstants.kPatternOrangeBlink,
            ()->layer3Pattern = LEDPattern.kOff
        ));

        swerve.isAligned.onTrue(runEnd(
            ()->layer4Pattern = LEDConstants.kPatternPurpleSolid,
            ()->layer4Pattern = LEDPattern.kOff
        ).withTimeout(0.75));
        swerve.isAligned.and(swerve.isAligning).whileTrue(runEnd(
            ()->layer4Pattern = LEDConstants.kPatternPurpleSolid,
            ()->layer4Pattern = LEDPattern.kOff
        ));
    }
    
    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }

    public void simulationPeriodic() {
        vision.simulationPeriodic(swerve.getState().Pose);
    }
}
/*TODO List:
 * automate algae shot (going to pose)
 * automate coral shot
 * l1 coral sequence
 * Autos: go to L2 after feedsequence
 * cleaner auto align logic
 * auto align + auto scoring?
*/