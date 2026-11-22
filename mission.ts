/**
 * บล็อคหมวด "ภารกิจส่งด่วน" — นำทางอัตโนมัติบนสนามภารกิจส่งด่วน
 * Uses the original extension's sensors/motors (ADCRead, motorGo, motorStop)
 * and its calibration arrays (Color_Line_All / Color_Background_All).
 */
//% color="#E67E22" icon="" block="ภารกิจส่งด่วน" weight=95
//% groups='["นำทาง", "ตั้งค่า"]'
namespace PTKidsMission {
    // ---- where the robot thinks it is ----
    let curNode = MissionNav.S
    let curHeading = 0

    // ---- settings (defaults from the plan) ----
    let runMin = 30
    let runMax = 60
    let runKP = 0.05
    let runKD = 0.1
    let turnSpeed = 50
    let turnBlindMs = 150
    let turnFwdMs = 120
    let uturnSpeed = 50
    let uturnBlindMs = 350
    let uturnBackMs = 150
    let arriveBoxMs = 0
    let arriveZoneMs = 300
    let arriveYellowMs = 250

    const FOLLOW_TIMEOUT = 8000
    const SPIN_TIMEOUT = 3000
    const ON = 500

    // ---- sensor layer: one read of all 6 channels per loop ----
    const vals = [0, 0, 0, 0, 0, 0]
    let prevError = 0
    let lastPos = 1500

    function readSensors(): void {
        const ch = [ADC_Read.ADC0, ADC_Read.ADC1, ADC_Read.ADC2, ADC_Read.ADC3, ADC_Read.ADC4, ADC_Read.ADC5]
        for (let i = 0; i < 6; i++) {
            let v = pins.map(PTKidsBITRobot.ADCRead(ch[i]), Color_Line_All[i], Color_Background_All[i], 1000, 0)
            if (v < 0) v = 0
            else if (v > 1000) v = 1000
            vals[i] = v
        }
    }

    function countOn(): number {
        let c = 0
        for (let i = 0; i < 6; i++) if (vals[i] > ON) c++
        return c
    }

    function isWide(): boolean { return countOn() >= 5 }

    function centerOnLine(): boolean { return vals[2] > ON || vals[3] > ON }

    /** line position from the 4 middle sensors, same scale as GETPosition (0..3000, 1500 = centre) */
    function position(): number {
        const thr = ADC_Version == 1 ? 200 : ON
        let sum = 0, weighted = 0, seen = false
        for (let i = 0; i < 4; i++) {
            const v = vals[i + 1]
            if (v > thr) seen = true
            weighted += v * i * 1000
            sum += v
        }
        if (!seen || sum == 0) return lastPos < 1500 ? 3000 : 0
        lastPos = weighted / sum
        return 3000 - lastPos
    }

    function pdStep(): void {
        const err = position() - 1500
        const pd = runKP * err + runKD * (err - prevError)
        prevError = err
        let l = runMin - pd, r = runMin + pd
        if (l > runMax) l = runMax
        else if (l < -runMax) l = -runMax
        if (r > runMax) r = runMax
        else if (r < -runMax) r = -runMax
        PTKidsBITRobot.motorGo(l, r)
    }

    function calibrated(): boolean {
        return Color_Line_All.length >= 6 && Color_Background_All.length >= 6
    }

    function failNotCalibrated(): void {
        PTKidsBITRobot.motorStop()
        music.playTone(262, 200)
        basic.showIcon(IconNames.Confused)
    }

    function fail(): void {
        PTKidsBITRobot.motorStop()
        music.playTone(262, 400)
        basic.showIcon(IconNames.No)
    }

    /** drive straight off a crossing / frame edge the sensors are sitting on */
    function leaveWide(): boolean {
        const t = control.millis()
        while (true) {
            readSensors()
            if (!isWide()) return true
            PTKidsBITRobot.motorGo(runMin, runMin)
            if (control.millis() - t > 1500) return false
        }
    }

    /** follow the line and stop on the n-th crossing */
    function follow(n: number): boolean {
        if (!leaveWide()) return false
        prevError = 0
        let c = 0
        const t = control.millis()
        while (true) {
            readSensors()
            if (isWide()) {
                c++
                if (c >= n) { PTKidsBITRobot.motorStop(); return true }
                if (!leaveWide()) return false
            } else pdStep()
            if (control.millis() - t > FOLLOW_TIMEOUT) return false
        }
    }

    /** spin (dir 0 = left, 1 = right) blind for blindMs, then until a centre sensor sees a line */
    //% blockHidden=true
    export function spinToLine(dir: number, speed: number, blindMs: number): boolean { return spin(dir, speed, blindMs) }

    function spin(dir: number, speed: number, blindMs: number): boolean {
        const l = dir == 0 ? -speed : speed
        PTKidsBITRobot.motorGo(l, -l)
        basic.pause(blindMs)
        const t = control.millis()
        // phase 1: leave the line the centre sensors may still be on
        while (true) {
            readSensors()
            if (!centerOnLine()) break
            PTKidsBITRobot.motorGo(l, -l)
            if (control.millis() - t > SPIN_TIMEOUT) return false
        }
        // phase 2: stop on the next line
        while (true) {
            readSensors()
            if (centerOnLine()) { PTKidsBITRobot.motorStop(); return true }
            PTKidsBITRobot.motorGo(l, -l)
            if (control.millis() - t > SPIN_TIMEOUT) return false
        }
    }

    function turn(dir: number): boolean {
        PTKidsBITRobot.motorGo(runMin, runMin)
        basic.pause(turnFwdMs)
        PTKidsBITRobot.motorStop()
        return spin(dir, turnSpeed, turnBlindMs)
    }

    function uturn(kind: number): boolean {
        let back = uturnBackMs
        if (kind == MissionNav.K_YELLOW) back = arriveYellowMs
        else if (kind == MissionNav.K_JUNCTION) back = 0
        if (back > 0) {
            PTKidsBITRobot.motorGo(-runMin, -runMin)
            basic.pause(back)
            PTKidsBITRobot.motorStop()
        }
        return spin(1, uturnSpeed, uturnBlindMs)
    }

    /** from inside START / a zone, drive out across the frame edge onto the line */
    function exitZone(): boolean {
        const t = control.millis()
        let seenWide = false
        while (true) {
            readSensors()
            const on = countOn()
            if (on >= 5) seenWide = true
            else if (seenWide) return true
            else if (on >= 1 && control.millis() - t > 300) return true // already on the line
            PTKidsBITRobot.motorGo(runMin, runMin)
            if (control.millis() - t > 2000) return false
        }
    }

    function arrive(arg: number): boolean {
        const kd = arg % 10
        const pass = Math.idiv(arg, 10)
        if (kd == MissionNav.K_YELLOW) {
            // stopped on the stop bar; the yellow box sits just past it (no line)
            PTKidsBITRobot.motorGo(runMin, runMin)
            basic.pause(arriveYellowMs)
            PTKidsBITRobot.motorStop()
            return true
        }
        if (!leaveWide()) return false
        prevError = 0
        let c = 0
        const t = control.millis()
        while (true) {
            readSensors()
            const on = countOn()
            if (on >= 5 && c < pass) {
                c++
                if (!leaveWide()) return false
            } else if (on >= 5 || on == 0) {
                // frame edge of the target square, or the end of the line
                PTKidsBITRobot.motorStop()
                break
            } else pdStep()
            if (control.millis() - t > FOLLOW_TIMEOUT) return false
        }
        const ms = kd == MissionNav.K_BOX ? arriveBoxMs : arriveZoneMs
        if (ms > 0) {
            PTKidsBITRobot.motorGo(runMin, runMin)
            basic.pause(ms)
            PTKidsBITRobot.motorStop()
        }
        return true
    }

    function run(acts: number[]): boolean {
        for (let i = 0; i < acts.length; i += 2) {
            const op = acts[i], arg = acts[i + 1]
            let ok = true
            if (op == MissionNav.OP_UTURN) ok = uturn(arg)
            else if (op == MissionNav.OP_EXIT) ok = exitZone()
            else if (op == MissionNav.OP_FOLLOW) ok = follow(arg)
            else if (op == MissionNav.OP_TURN) ok = turn(arg)
            else if (op == MissionNav.OP_ARRIVE) ok = arrive(arg)
            if (!ok) return false
        }
        return true
    }

    // ================= blocks =================

    /**
     * วิ่งจากจุดแรกไปจุดที่สอง (ตั้งตำแหน่งเริ่มให้เอง)
     */
    //% block="ไปจาก $from ถึง $to"
    //% group="นำทาง" weight=100
    export function goFromTo(from: MissionPoint, to: MissionPoint): void {
        setPosition(from)
        goTo(to)
    }

    /**
     * วิ่งจากตำแหน่งที่หุ่นจำไว้ล่าสุด ไปยังจุดที่เลือก
     */
    //% block="ไปที่ $to"
    //% group="นำทาง" weight=90
    export function goTo(to: MissionPoint): void {
        if (Calibrating) return   // a button press during SensorCalibrate must not drive the robot
        if (!calibrated()) { failNotCalibrated(); return }
        const goal = MissionNav.pointNode(to)
        const acts = MissionNav.plan(curNode, curHeading, goal)
        if (acts.length == 0) return
        if (run(acts)) {
            curNode = goal
            curHeading = MissionNav.finalHeading()
        } else fail()
    }

    /**
     * บอกหุ่นว่าตอนนี้อยู่จุดไหน (ใช้หลัง RETRY: ตั้งที่ START)
     */
    //% block="ตั้งตำแหน่งหุ่นที่ $p"
    //% group="นำทาง" weight=80
    export function setPosition(p: MissionPoint): void {
        curNode = MissionNav.pointNode(p)
        curHeading = MissionNav.startHeading(p)
    }

    /**
     * จุดที่หุ่นคิดว่าตัวเองอยู่ (เลขตามลำดับในเมนู)
     */
    //% block="ตำแหน่งปัจจุบัน"
    //% group="นำทาง" weight=70
    export function currentPosition(): number {
        return curNode < 16 ? curNode : -1
    }

    /**
     * ค่าวิ่งตามเส้นที่บล็อคนำทางใช้
     */
    //% block="ตั้งค่าการวิ่ง ความเร็วต่ำสุด $min สูงสุด $max KP $kp KD $kd"
    //% min.defl=30 max.defl=60 kp.defl=0.05 kd.defl=0.1
    //% min.min=0 min.max=100 max.min=0 max.max=100
    //% group="ตั้งค่า" weight=60 inlineInputMode=inline
    export function setRun(min: number, max: number, kp: number, kd: number): void {
        runMin = min; runMax = max; runKP = kp; runKD = kd
    }

    /**
     * ค่าเลี้ยว 90° ที่แยก
     */
    //% block="ตั้งค่าการเลี้ยว ความเร็ว $speed เวลาหมุนเร็ว $blindMs ms เดินหน้าก่อนเลี้ยว $fwdMs ms"
    //% speed.defl=50 blindMs.defl=150 fwdMs.defl=120
    //% speed.min=0 speed.max=100
    //% group="ตั้งค่า" weight=50 inlineInputMode=inline
    export function setTurn(speed: number, blindMs: number, fwdMs: number): void {
        turnSpeed = speed; turnBlindMs = blindMs; turnFwdMs = fwdMs
    }

    /**
     * ค่าหมุนกลับหัว 180°
     */
    //% block="ตั้งค่ากลับหัว ความเร็ว $speed เวลาหมุนเร็ว $blindMs ms ถอยก่อนหมุน $backMs ms"
    //% speed.defl=50 blindMs.defl=350 backMs.defl=150
    //% speed.min=0 speed.max=100
    //% group="ตั้งค่า" weight=40 inlineInputMode=inline
    export function setUturn(speed: number, blindMs: number, backMs: number): void {
        uturnSpeed = speed; uturnBlindMs = blindMs; uturnBackMs = backMs
    }

    /**
     * เวลาเดินต่อหลังหยุดที่ปลายทาง
     */
    //% block="ตั้งค่าการเข้าจุดปลายทาง Box $boxMs ms START/ช่องสี $zoneMs ms Box เหลือง $yellowMs ms"
    //% boxMs.defl=0 zoneMs.defl=300 yellowMs.defl=250
    //% group="ตั้งค่า" weight=30 inlineInputMode=inline
    export function setArrive(boxMs: number, zoneMs: number, yellowMs: number): void {
        arriveBoxMs = boxMs; arriveZoneMs = zoneMs; arriveYellowMs = yellowMs
    }
}
