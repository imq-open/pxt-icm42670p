
// serial.setTxBufferSize(128)

icm42670p.setSlaveAddress(icm42670p.SlaveAddress.AD0_0)
icm42670p.setInt1Pin(DigitalPin.P16)

control.onEvent(EventBusSource.ICM42670P_DEVICE_ID, EventBusValue.ICM42670P_EVT_DATA_UPDATED, function () {

    let ang = icm42670p.getEulerAngles()
    let temp = icm42670p.getTempDegC()
    serial.writeString("Pitch: " + ang.getNumber(NumberFormat.Float32LE, 4)
        + ", Roll: " + ang.getNumber(NumberFormat.Float32LE, 8)
        + ", Temp: " + temp + " deg C\n"
    )
})

basic.forever(function () {
    let rc = icm42670p.poll()
    serial.writeString("poll break with code " + rc + "\n")
})
