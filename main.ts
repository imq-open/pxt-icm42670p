
icm42670p.setSlaveAddress(icm42670p.SlaveAddress.AD0_0)
icm42670p.setInt1Pin(DigitalPin.P16)

control.onEvent(EventBusSource.ICM42670P_DEVICE_ID, EventBusValue.ICM42670P_EVT_DATA_UPDATED, function () {
    let accel = icm42670p.getAccelData(false)
    serial.writeString("Accel: " + accel.getNumber(NumberFormat.Float32LE, 0))
    serial.writeString(", " + accel.getNumber(NumberFormat.Float32LE, 4))
    serial.writeString(", " + accel.getNumber(NumberFormat.Float32LE, 8))
    serial.writeString("\n")

    let gyro = icm42670p.getGyroData(false)
    serial.writeString("Gyro: " + accel.getNumber(NumberFormat.Float32LE, 0))
    serial.writeString(", " + accel.getNumber(NumberFormat.Float32LE, 4))
    serial.writeString(", " + accel.getNumber(NumberFormat.Float32LE, 8))
    serial.writeString("\n")

    serial.writeString("Temp: " + icm42670p.getTempData(false) + "\n")

})

basic.forever(function () {
    let rc = icm42670p.poll()
    serial.writeString("poll loop breaks with error: " + rc + "\n")
})
