//package org.firstinspires.ftc.teamcode.Testing.Misc;
//
//import com.qualcomm.robotcore.hardware.HardwareDevice;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
//import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
//import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
//import com.qualcomm.robotcore.util.TypeConversion;
//
//@I2cDeviceType
//@DeviceProperties(name = "Adafruit LED", xmlTag = "HT16K33")
//public class SimpleI2C extends I2cDeviceSynchDevice<I2cDeviceSynch>
//{
//
//
//    protected void writeShort(final Register reg, short value)
//    {
//        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
//    }
//
//
//    @Override
//    public HardwareDevice.Manufacturer getManufacturer()
//    {
//
//        return Manufacturer.Adafruit;
//    }
//
//    @Override
//    protected synchronized boolean doInitialize()
//    {
//        return true;
//    }
//
//    @Override
//    public String getDeviceName()
//    {
//
//        return "Adafruit MCP9808 Temperature Sensor";
//    }
//
//    public SimpleI2C(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
//    {
//        super(deviceClient, deviceClientIsOwned);
//
//        super.registerArmingStateCallback(false);
//        this.deviceClient.engage();
//    }
//}