package org.firstinspires.ftc.teamcode.Testing.Misc;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import java.util.Arrays;

/*
 * Created by Dryw Wade
 *
 * Driver for Adafruit's MCP9808 temperature sensor
 *
 * This version of the driver does not make use of the I2C device with parameters. This means the
 * settings for the configuration register are hard coded and cannot be changed by the user, nor can
 * they be different for each OpMode.
 */
@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
// Both driver classes cannot register the sensor at the same time. One driver should have the
// sensor registered, and the other should be commented out
@I2cDeviceType
@DeviceProperties(name = "LEDBackpack", description = "an MCP9808 temperature sensor", xmlTag = "LEDBackpack")
public class AdafruitLEDBackpack extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

//    public void setOscillator(boolean on)
//    {
//        writeShort(Register.OSCILLATION, (short) 0x01);
//    }

    public void setLED(int index, boolean on)
    {
        byte[] data = new byte[1];
        data[0] = (byte) (on ? 1 : 0);
        deviceClient.write(Register.SENDING.bVal + index, data);
    }

    public void allOn() {
        byte[] array = new byte[8];
        Arrays.fill(array, (byte) 0xFF); // Fill the array with 0xFF (all 1s)
        deviceClient.write(Register.SENDING.bVal, array);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Read and Write Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    protected void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Registers and Config Settings
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public enum Register
    {
        OSCILLATION(0x01),
        SENDING(0x02);

//        FIRST(0),
//        CONFIGURATION(0x01),
//        T_LIMIT_UPPER(0x02),
//        T_LIMIT_LOWER(0x03),
//        T_LIMIT_CRITICAL(0x04),
//        TEMPERATURE(0x05),
//        MANUFACTURER_ID(0x06),
//        DEVICE_ID_REVISION(0x07),
//        RESOLUTION(0x08),
//        LAST(RESOLUTION.bVal);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }



    // More settings are available on the sensor, but not included here. Could be added later

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Construction and Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x70);

    public AdafruitLEDBackpack(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }

    @Override
    protected synchronized boolean doInitialize()
    {

        writeShort(Register.OSCILLATION, (short) 0x20);

        return true;
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName()
    {
        return "Adafruit LED Backpack";
    }
}