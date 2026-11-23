// ตัวอย่างโปรแกรมแข่ง: ภารกิจอัตโนมัติ (ไม่ถูกคอมไพล์เมื่อใช้เป็น Extension)
// 1) เปิดเครื่อง → คาลิเบรต  2) ดูสนาม แล้วใส่สีกล่องด้วยปุ่ม A=ดำ B=ขาว
// 3) วางหุ่นที่ START กด B = เริ่ม / RETRY (ทำต่อจากกล่องที่ยังไม่ส่ง)
// 4) กด A+B = รอบใหม่ (ใส่สีใหม่ เริ่มนับกล่องใหม่)
let ready = false
PTKidsBITRobot.SensorCalibrate()
PTKidsMission.setRun(30, 60, 0.05, 0.1)
PTKidsMission.setGripper(Servo_Write.P8, 90, 0, Servo_Write.P12, 180, 0)
PTKidsMission.inputBoxColors()
ready = true
input.onButtonPressed(Button.B, function () {
    if (!ready) return
    PTKidsMission.startAutoMission()
    basic.showNumber(PTKidsMission.boxesDone())
})
input.onButtonPressed(Button.AB, function () {
    ready = false
    PTKidsMission.resetAutoMission()
    PTKidsMission.inputBoxColors()
    ready = true
})
