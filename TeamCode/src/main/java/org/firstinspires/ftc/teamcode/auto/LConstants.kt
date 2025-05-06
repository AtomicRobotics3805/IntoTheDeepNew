package org.firstinspires.ftc.teamcode.auto

import com.pedropathing.localization.Encoder
import com.pedropathing.localization.constants.TwoWheelConstants
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot


class LConstants {
    init {
        TwoWheelConstants.forwardTicksToInches = 0.002992230485396
        TwoWheelConstants.strafeTicksToInches = 0.003017977775679
        TwoWheelConstants.forwardY = 4.4469
        TwoWheelConstants.strafeX = -5.3917
        TwoWheelConstants.forwardEncoder_HardwareMapName = "RB"
        TwoWheelConstants.strafeEncoder_HardwareMapName = "LB"
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD
        TwoWheelConstants.IMU_HardwareMapName = "imu"
        TwoWheelConstants.IMU_Orientation = RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        )
    }
}