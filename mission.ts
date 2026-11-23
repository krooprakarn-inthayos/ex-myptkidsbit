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
    let arriveStartMs = 300   // START: drive fully into the frame
    let arriveZoneMs = 0      // white / black zone: stop at the frame edge
    let goalNode = -1
    let arriveYellowMs = 250
    let approachSpeed = 30    // last segment into a box (from its second-last crossing)
    let approachStopMs = 0    // > 0: stop this long after the last crossing before a box
    let showStopReason = false
    let sideTurn = true       // use ADC6/ADC7 to find the wheel position before a 90° turn
    let sideFwdMs = 100       // drive on after a side sensor sees the crossing
    // gripper handling on the turn into a box (off until setGripper is used)
    let gripOn = false
    let armServo = Servo_Write.P8
    let armUp = 90
    let armDown = 0
    let gripServo = Servo_Write.P12
    let gripOpen = 180
    let gripClose = 0
    const SERVO_MS = 300

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
        pdStepAt(runMin, runMax)
    }

    function pdStepAt(base: number, max: number): void {
        const err = position() - 1500
        const pd = runKP * err + runKD * (err - prevError)
        prevError = err
        let l = base - pd, r = base + pd
        if (l > max) l = max
        else if (l < -max) l = -max
        if (r > max) r = max
        else if (r < -max) r = -max
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
        return leaveWideAt(runMin)
    }

    function leaveWideAt(v: number): boolean {
        const t = control.millis()
        while (true) {
            readSensors()
            if (!isWide()) return true
            PTKidsBITRobot.motorGo(v, v)
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
            position()   // keep "which side the line was last seen" fresh for the line follower
            if (centerOnLine()) { PTKidsBITRobot.motorStop(); return true }
            PTKidsBITRobot.motorGo(l, -l)
            if (control.millis() - t > SPIN_TIMEOUT) return false
        }
    }

    function sideCalibrated(): boolean {
        return Side_Line.length >= 2 && Side_Ground.length >= 2
    }

    /** true when either side sensor (ADC6 left, ADC7 right) sees black */
    function sideOnLine(): boolean {
        const l = pins.map(PTKidsBITRobot.ADCRead(ADC_Read.ADC6), Side_Line[0], Side_Ground[0], 1000, 0)
        const r = pins.map(PTKidsBITRobot.ADCRead(ADC_Read.ADC7), Side_Line[1], Side_Ground[1], 1000, 0)
        return l > ON || r > ON
    }

    function turn(dir: number): boolean {
        if (sideTurn && sideCalibrated()) {
            // drive on until a side sensor reaches the crossing, then a short fixed distance to the wheels
            const t = control.millis()
            while (!sideOnLine()) {
                PTKidsBITRobot.motorGo(runMin, runMin)
                if (control.millis() - t > 1000) break   // missed it: turn from here rather than run away
            }
            PTKidsBITRobot.motorStop()
            nudge(sideFwdMs)
        } else {
            nudge(turnFwdMs)
        }
        return spin(dir, turnSpeed, turnBlindMs)
    }

    /**
     * reverse (sensors at the front) until they sit on a crossing / frame edge.
     * ignoreMs: at the start, ignore a crossing the sensors may already be on (the box's small frame)
     */
    function backToCrossing(ignoreMs: number): boolean {
        const t = control.millis()
        while (true) {
            readSensors()
            if (isWide() && control.millis() - t >= ignoreMs) { PTKidsBITRobot.motorStop(); return true }
            PTKidsBITRobot.motorGo(-runMin, -runMin)
            if (control.millis() - t > 3000) return false
        }
    }

    /**
     * reverse out of START / a zone: the frame edge is ignored until the plain line has been seen
     * for a few reads, then stop on the nearest crossing.
     */
    function backOutOfZone(): boolean {
        const t = control.millis()
        let lineReads = 0
        while (true) {
            readSensors()
            const on = countOn()
            if (on >= 1 && on <= 3 && centerOnLine()) lineReads++
            else if (on < 5) lineReads = 0
            if (on >= 5 && lineReads >= 3) { PTKidsBITRobot.motorStop(); return true }
            PTKidsBITRobot.motorGo(-runMin, -runMin)
            if (control.millis() - t > 3000) return false
        }
    }

    function uturn(kind: number): boolean {
        // START / zone: come back to the frame edge first, then spin like on a crossing
        if (kind == MissionNav.K_ZONE && !backToCrossing(0)) return false
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
            nudge(arriveYellowMs)
            return true
        }
        const toBox = kd == MissionNav.K_BOX
        const slow = Math.min(approachSpeed, runMin)
        // into a box with nothing to pass: slow from the start
        if (!leaveWideAt(toBox && pass == 0 ? slow : runMin)) return false
        prevError = 0
        let c = 0
        let seenLine = false
        let lostReads = 0
        const t = control.millis()
        let tLast = t        // when the robot left the last crossing before the box
        while (true) {
            readSensors()
            const on = countOn()
            if (centerOnLine()) seenLine = true
            lostReads = on == 0 ? lostReads + 1 : 0
            // into a box: slow down from the second-last crossing; after the last one the small
            // box frame (≥ 4 sensors) or the approach timer ends the run
            const slowRun = toBox && c >= pass - 1
            const boxRun = toBox && c >= pass
            if (on >= 5 && c < pass) {
                c++
                if (!leaveWideAt(slowRun ? slow : runMin)) return false
                if (c >= pass) tLast = control.millis()
            } else {
                // why we stop: 0 frame edge, 1 end of the line, 2 approach timer (only after the last crossing:
                // there is no line end between crossings, a lost line there is just a wobble)
                const pastLast = c >= pass
                let why = -1
                if (on >= 5 || (boxRun && on >= 4)) why = 0
                else if (pastLast && seenLine && lostReads >= 3) why = 1
                else if (boxRun && approachStopMs > 0 && control.millis() - tLast >= approachStopMs) why = 2
                if (why >= 0) {
                    PTKidsBITRobot.motorStop()
                    showWhy(why)
                    break
                }
                if (slowRun) pdStepAt(slow, runMax)
                else pdStep()
            }
            if (control.millis() - t > FOLLOW_TIMEOUT) return false
        }
        let ms = arriveBoxMs
        if (kd == MissionNav.K_ZONE) ms = goalNode == MissionNav.S ? arriveStartMs : arriveZoneMs
        nudge(ms)
        return true
    }

    /** debug: light one LED in the top row — left = frame edge, middle = end of line, right = timer */
    function showWhy(why: number): void {
        if (!showStopReason) return
        basic.clearScreen()
        led.plot(why * 2, 0)
    }

    /** drive forward ms (> 0) or back -ms (< 0) at the base speed, then stop */
    function nudge(ms: number): void {
        if (ms == 0) return
        const v = ms > 0 ? runMin : -runMin
        PTKidsBITRobot.motorGo(v, v)
        basic.pause(Math.abs(ms))
        PTKidsBITRobot.motorStop()
    }

    /**
     * turn towards a box that is right after this crossing: the open gripper would sweep into it.
     * lift + close, turn, back the sensors onto the crossing, then lower + open before driving in.
     */
    function turnIntoBox(dir: number): boolean {
        PTKidsBITRobot.servoWrite(armServo, armUp)
        PTKidsBITRobot.servoWrite(gripServo, gripClose)
        basic.pause(SERVO_MS)
        if (!turn(dir)) return false
        const ok = backToCrossing(0)
        PTKidsBITRobot.servoWrite(armServo, armDown)
        PTKidsBITRobot.servoWrite(gripServo, gripOpen)
        basic.pause(SERVO_MS)
        return ok
    }

    function run(acts: number[]): boolean {
        for (let i = 0; i < acts.length; i += 2) {
            const op = acts[i], arg = acts[i + 1]
            let ok = true
            const nextIsBox = i + 3 < acts.length && acts[i + 2] == MissionNav.OP_ARRIVE && acts[i + 3] == MissionNav.K_BOX
            if (op == MissionNav.OP_TURN && gripOn && nextIsBox) ok = turnIntoBox(arg)
            else if (op == MissionNav.OP_UTURN) ok = uturn(arg)
            else if (op == MissionNav.OP_EXIT) ok = exitZone()
            else if (op == MissionNav.OP_FOLLOW) ok = follow(arg)
            else if (op == MissionNav.OP_TURN) ok = turn(arg)
            else if (op == MissionNav.OP_ARRIVE) ok = arrive(arg)
            else if (op == MissionNav.OP_BACK) ok = arg == MissionNav.K_ZONE ? backOutOfZone() : backToCrossing(uturnBackMs)
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
        goalNode = goal
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
     * ค่าหมุนกลับหัว 180° และการถอยออกจาก Box
     * backMs = ช่วงแรกของการถอยที่ไม่นับขอบกรอบ Box (กันหยุดผิดที่ขอบกรอบ)
     */
    //% block="ตั้งค่ากลับหัว ความเร็ว $speed เวลาหมุนเร็ว $blindMs ms ถอยออกจาก Box ไม่นับขอบกรอบช่วงแรก $backMs ms"
    //% speed.defl=50 blindMs.defl=350 backMs.defl=150
    //% speed.min=0 speed.max=100
    //% group="ตั้งค่า" weight=40 inlineInputMode=inline
    export function setUturn(speed: number, blindMs: number, backMs: number): void {
        uturnSpeed = speed; uturnBlindMs = blindMs; uturnBackMs = backMs
    }

    /**
     * เวลาเดินต่อหลังหยุดที่ปลายทาง (ค่าติดลบ = ถอยกลับ)
     */
    //% block="ตั้งค่าการเข้าจุดปลายทาง Box $boxMs ms START $startMs ms Box เหลือง $yellowMs ms"
    //% boxMs.defl=0 startMs.defl=300 yellowMs.defl=250
    //% group="ตั้งค่า" weight=30 inlineInputMode=inline
    export function setArrive(boxMs: number, startMs: number, yellowMs: number): void {
        arriveBoxMs = boxMs; arriveStartMs = startMs; arriveYellowMs = yellowMs
    }

    /**
     * เวลาเดินต่อหลังเจอขอบช่องขาว/ช่องดำ: 0 = หยุดที่ขอบ, ค่าบวก = เดินเข้าไป, ค่าติดลบ = ถอยกลับ
     */
    //% block="ตั้งค่าการเข้าช่องขาว/ช่องดำ $ms ms"
    //% ms.defl=0
    //% group="ตั้งค่า" weight=25
    export function setArriveZone(ms: number): void {
        arriveZoneMs = ms
    }

    /**
     * ใส่ค่าคาลิเบรตเซนเซอร์ข้าง ADC6 (ซ้าย) และ ADC7 (ขวา) เอง (ค่าจาก Serial หลัง SensorCalibrate)
     */
    //% block="ตั้งค่าเซนเซอร์ข้าง ADC6 เส้น $line6 พื้น $ground6 ADC7 เส้น $line7 พื้น $ground7"
    //% group="ตั้งค่า" weight=20 inlineInputMode=inline
    export function setSideSensors(line6: number, ground6: number, line7: number, ground7: number): void {
        Side_Line = [line6, line7]
        Side_Ground = [ground6, ground7]
    }

    /**
     * เลี้ยวด้วยเซนเซอร์ข้าง: เดินหน้าจนเซนเซอร์ข้างเห็นเส้นตัด แล้วเดินต่อ ms ก่อนหมุน
     * (ปิด หรือยังไม่คาลิเบรต = ใช้เวลา "เดินหน้าก่อนเลี้ยว" ใน ตั้งค่าการเลี้ยว แทน)
     */
    //% block="ตั้งค่าเลี้ยวด้วยเซนเซอร์ข้าง ใช้ $on เดินต่อ $ms ms"
    //% on.defl=true ms.defl=100
    //% group="ตั้งค่า" weight=15 inlineInputMode=inline
    export function setSideTurn(on: boolean, ms: number): void {
        sideTurn = on
        sideFwdMs = ms
    }

    /**
     * (ไม่จำเป็นตั้งแต่ 0.4.0: หุ่นเข้าหา Box แบบทางตรงเสมอ — เก็บไว้ให้โปรแกรมเก่าคอมไพล์ได้)
     * กริปเปอร์อัตโนมัติ: ตอนเลี้ยวเข้าหา Box ที่อยู่ถัดจากแยก หุ่นจะยกแขน+หุบมือจับ เลี้ยว ถอยกลับมาที่แยก
     * แล้วลดแขน+อ้ามือจับ ก่อนวิ่งเข้าหา Box (กันกริปเปอร์กวาดชนกล่อง)
     */
    //% block="ตั้งค่ากริปเปอร์ แขน $arm ยกขึ้น $up ลดลง $down มือจับ $grip อ้า $open หนีบ $close"
    //% arm.defl=Servo_Write.P8 up.defl=90 down.defl=0 grip.defl=Servo_Write.P12 open.defl=180 close.defl=0
    //% up.min=0 up.max=180 down.min=0 down.max=180 open.min=0 open.max=180 close.min=0 close.max=180
    //% group="ตั้งค่า" weight=10 inlineInputMode=inline blockHidden=true
    export function setGripper(arm: Servo_Write, up: number, down: number, grip: Servo_Write, open: number, close: number): void {
        armServo = arm; armUp = up; armDown = down
        gripServo = grip; gripOpen = open; gripClose = close
        gripOn = true
    }

    /**
     * ปิดกริปเปอร์อัตโนมัติ
     */
    //% block="ปิดกริปเปอร์อัตโนมัติ"
    //% group="ตั้งค่า" weight=5 blockHidden=true
    export function clearGripper(): void {
        gripOn = false
    }

    /**
     * ความเร็วช่วงสุดท้ายก่อนถึง Box (หลังผ่านแยกสุดท้าย) ยิ่งช้า ยิ่งหยุดแม่น ไม่ชนกล่อง
     */
    //% block="ตั้งค่าความเร็วเข้า Box $speed"
    //% speed.defl=30 speed.min=10 speed.max=100
    //% group="ตั้งค่า" weight=28
    export function setApproach(speed: number): void {
        approachSpeed = speed
    }

    /**
     * หยุดตามเวลาหลังผ่านแยกสุดท้ายก่อนถึง Box (0 = หยุดเมื่อเซนเซอร์เห็นขอบกรอบ Box)
     * ใช้เมื่อหุ่นยังดันกล่อง: ตั้งให้หยุดตอนกริปเปอร์ครอบกล่องพอดี
     */
    //% block="ตั้งค่าหยุดหลังผ่านแยกสุดท้ายก่อนถึง Box $ms ms"
    //% ms.defl=0 ms.min=0
    //% group="ตั้งค่า" weight=27
    export function setApproachStop(ms: number): void {
        approachStopMs = ms
    }

    /**
     * แสดงเหตุผลที่หุ่นหยุดตอนถึงปลายทาง (ไฟแถวบนสุด): ซ้าย = เจอขอบกรอบ, กลาง = สุดปลายเส้น, ขวา = หมดเวลาเข้า Box
     */
    //% block="แสดงเหตุผลการหยุด $on"
    //% on.defl=true
    //% group="ตั้งค่า" weight=2
    export function setShowStopReason(on: boolean): void {
        showStopReason = on
    }
}
