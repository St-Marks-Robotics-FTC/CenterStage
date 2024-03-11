package org.firstinspires.ftc.teamcode.Testing.Misc;

import android.graphics.Color;
import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType()
@DeviceProperties(name = "QWIIC LED Stick", description = "Sparkfun QWIIC LED Stick", xmlTag = "QWIIC_LED_STICK")
public class OpenI2C extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {

    private enum Commands {
        WRITE_OSCILLATOR(0x21),
        WRITE_LED(0X00);

        int bVal;

        Commands(int bVal) {
            this.bVal = bVal;
        }
    }

//    public void setLED(int index) {
//        writeI2C(Commands.WRITE_LED, new byte[]{(byte) index});
//    }

    private void setOscillator() {
        writeI2C(Commands.WRITE_OSCILLATOR, new byte[]{0x01});
    }

    public void allOn() {
        int[] array = new int[8];
        for (int i = 0; i < 8; i++) {
            array[i] = 1;
        }
        sendSegment(Commands.WRITE_LED, array);
    }

    /**
     * Send a segment of the LED array
     *
     * @param cmd    command to send
     * @param array  the values (limited from 0..255)
     */
    private void sendSegment(Commands cmd, int[] array) {
        byte[] data = new byte[8];

        for (int i = 0; i < 8; i++) { // Iterate only 8 times
            data[i] = (byte) array[i];
        }
        writeI2C(cmd, data);
    }


    private void writeI2C(Commands cmd, byte[] data) {
        deviceClient.write(cmd.bVal, data, I2cWaitControl.WRITTEN);
    }


    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize() {
        setOscillator();
        return true;
    }

    @Override
    public String getDeviceName() {
        return "Qwiic LED Strip";
    }

    private final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x70);

    public OpenI2C(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
    }

}