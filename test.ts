// ตัวอย่าง: คาลิเบรต แล้วกด B เพื่อวิ่งไปเก็บ Box UL และกลับ START
// (ไม่ถูกคอมไพล์เมื่อใช้เป็น Extension)
PTKidsBITRobot.SensorCalibrate()
PTKidsMission.setPosition(MissionPoint.START)
basic.showIcon(IconNames.Yes)
input.onButtonPressed(Button.B, function () {
    PTKidsBITRobot.servoWrite(Servo_Write.P12, 90)
    PTKidsMission.goTo(MissionPoint.UL)
    PTKidsBITRobot.servoWrite(Servo_Write.P12, 20)
    PTKidsMission.goTo(MissionPoint.START)
    PTKidsBITRobot.servoWrite(Servo_Write.P12, 90)
    basic.showNumber(PTKidsMission.currentPosition())
})
input.onButtonPressed(Button.A, function () {
    PTKidsMission.setPosition(MissionPoint.START)
})
