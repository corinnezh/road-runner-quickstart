package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

public class Arm {
    private DcMotorEx lowerArmMotor;
    private DcMotorEx upperArmMotor;
    private AnalogInput lowerArmEncoder;
    private AnalogInput upperArmEncoder;


    public Arm(HardwareMap hardwareMap) {
        lowerArmMotor = hardwareMap.get(DcMotorEx.class, "lowerArmMotor");
        lowerArmMotor.setDirection(DcMotorEx.Direction.FORWARD);

        upperArmMotor = hardwareMap.get(DcMotorEx.class, "upperArmMotor");
        upperArmMotor.setDirection(CRServoImplEx.Direction.FORWARD);

        lowerArmEncoder = hardwareMap.get(AnalogInput.class, "lowerArmMotor");
        upperArmEncoder = hardwareMap.get(AnalogInput.class, "upperArmMotor");

    }
    

    public static class ArmPosition { //don't have delay in outtake but it might not matter cause so light and small
        final double lowerArmAngle, upperArmAngle;
        final boolean atScoringPosition;
        final double outtakeSpeed;

        public ArmPosition(double lowerArmAngle, double upperArmAngle, boolean atScoringPosition, double outtakeSpeed) {
            this.lowerArmAngle = lowerArmAngle;
            this.upperArmAngle = upperArmAngle;

            this.atScoringPosition = atScoringPosition;
            this.outtakeSpeed = outtakeSpeed;

        }

    }

}
