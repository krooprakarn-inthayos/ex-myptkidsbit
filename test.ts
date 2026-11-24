// โปรแกรมแข่ง: ภารกิจอัตโนมัติด้วยค่าเริ่มต้นสำหรับแข่ง (ไม่ถูกคอมไพล์เมื่อใช้เป็น Extension)
// ค่าตั้งทั้งหมดถูกตั้งไว้แล้วตั้งแต่เปิดเครื่อง — ใส่บล็อคตั้งค่าเฉพาะค่าที่อยากเปลี่ยน
// 1) เปิดเครื่อง → คาลิเบรต  2) ดูสนาม ใส่สีกล่องด้วยปุ่ม A=ดำ B=ขาว (ได้ยินเพลง = พร้อม)
// 3) วางหุ่นที่ START กด B = เริ่ม / RETRY  4) กด A+B = รอบใหม่
let ready = false
let t0 = 0
PTKidsBITRobot.SensorCalibrate()
PTKidsMission.inputBoxColors()
ready = true
input.onButtonPressed(Button.B, function () {
    if (!ready) return
    ready = false
    if (PTKidsMission.boxesDone() == 0) t0 = control.millis()
    PTKidsMission.startAutoMission()
    if (PTKidsMission.boxesDone() == 8 && PTKidsMission.currentPosition() == MissionPoint.START) {
        basic.showNumber(Math.round((control.millis() - t0) / 1000))
    } else {
        basic.showNumber(PTKidsMission.boxesDone())
    }
    ready = true
})
input.onButtonPressed(Button.AB, function () {
    if (!ready) return
    ready = false
    PTKidsMission.resetAutoMission()
    PTKidsMission.inputBoxColors()
    ready = true
})
