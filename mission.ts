/** สีของกล่อง */
enum BoxColor {
    //% block="ดำ"
    Black = 0,
    //% block="ขาว"
    White = 1
}

/**
 * บล็อคหมวด "ภารกิจส่งด่วน" — นำทางอัตโนมัติบนสนามภารกิจส่งด่วน
 * Uses the original extension's sensors/motors (ADCRead, motorGo, motorStop)
 * and its calibration arrays (Color_Line_All / Color_Background_All).
 */
//% color="#E67E22" icon="" block="ภารกิจส่งด่วน" weight=95
//% groups='["ภารกิจอัตโนมัติ", "เขียนเส้นทางเอง", "หยุด / ทำต่อ", "นำทาง", "ตั้งค่า"]'
namespace PTKidsMission {
    // ---- where the robot thinks it is ----
    let curNode = MissionNav.S
    let curHeading = 0

    // ---- settings: power-on values = the tuned competition set (keep in step with resetSettings) ----
    let runMin = 60
    let runMax = 100
    let runKP = 0.046
    let runKD = 0.25
    let turnSpeed = 60
    let turnBlindMs = 125
    let turnFwdMs = 120
    let uturnSpeed = 60
    let uturnBlindMs = 290
    let uturnBackMs = 150
    let arriveBoxMs = 40
    let arriveStartMs = 250   // START: drive fully into the frame
    let arriveZoneMs = 0      // white / black zone: stop at the frame edge
    let goalNode = -1
    let arriveYellowMs = 50
    let approachSpeed = 40    // last segment into a box (from its second-last crossing)
    let approachStopMs = 0    // > 0: stop this long after the last crossing before a box
    let showStopReason = false
    let zoneBackSpeed = 65    // > 0: speed reversing out of a colour zone / START (0 = min(runMin, approach))
    let crossSpeed = 50       // > 0: speed for timed moves and for approaching a stop/turn crossing (0 = runMin)
    let fastSpeed = 100        // > runMin: base speed on straights between crossings (0 = off)
    let brakeCm = 14         // slow to runMin this far before the crossing where the robot stops
    const CMPMS_START = 0.15 // 150 cm/s: assumed speed before any measurement (high = brakes early, safe)
    let cmPerMs = CMPMS_START // learned travel speed at fastSpeed, in cm per millisecond
    let speedLearned = false
    const RAMP = 5           // base speed step per loop when speeding up
    let gripHoldMs = 100     // wait after closing before driving off
    let gripReleaseMs = 50  // wait after opening before driving off
    let boxStopAtCrossing = true
    // per-box forward time after stopping on the crossing (box index 0 UL … 7 B2); used where set
    let boxArriveMs = [0, 0, 0, 0, 0, 0, 0, 0]
    let boxArriveSet = [false, false, false, false, true, true, false, false]   // UR, DR: 0 ms (closer to their crossing)   // stop on the crossing right before a box (gripper reaches the box from there)
    let sideTurn = true       // use ADC6/ADC7 to find the wheel position before a 90° turn
    let sideFwdMs = 100       // drive on after a side sensor sees the crossing
    // gripper handling on the turn into a box (off until setGripper is used)
    let gripOn = false
    let armServo = Servo_Write.P8
    let armUp = 24
    let armDown = 110
    let gripServo = Servo_Write.P12
    let gripOpen = 180
    let gripClose = 0
    const SERVO_MS = 300
    let smoothTurn = true          // turns without stopping on the crossing or after the spin
    let pendingArm = false          // lower the arm during the next spin (turn into a colour zone)
    let lowerArmBeforeZone = true   // lower the arm on the crossing before the white/black zone
    let lowerArmBeforeBox = false   // lower the arm (and open) at the start of the approach into a box

    const FOLLOW_TIMEOUT = 8000
    const SPIN_TIMEOUT = 3000
    const ON = 500
    // after leaving a crossing, the next crossing / frame edge only counts once the robot has seen
    // a normal line (1–3 sensors) this many reads in a row: a wobble back onto the same crossing is not a new one
    const CLEAR_READS = 2

    // ---- sensor layer: one read of all 6 channels per loop ----
    const vals = [0, 0, 0, 0, 0, 0]
    let prevError = 0
    let lastPos = 1500

    // ---- stop button: A pressed while the robot drives stops it at once ----
    let stopRequested = false
    let moving = false            // inside a drive (goTo / mission step)
    let missionRunning = false    // inside runTrips / startAutoMission
    let stopButtonA = true
    let swallowUndo = false       // the click of the A press that stopped the robot must not undo a trip
    let stopAt = -100000

    /** true once a stop was asked for; also polls button A while driving */
    function pollStop(): boolean {
        if (moving && !stopRequested && stopButtonA && input.buttonIsPressed(Button.A)) {
            stopRequested = true
            swallowUndo = true
            stopAt = control.millis()
        }
        if (stopRequested) PTKidsBITRobot.motorStop()
        return stopRequested
    }

    function readSensors(): void {
        pollStop()
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
            if (stopRequested || control.millis() - t > 1500) return false
        }
    }

    /** after leaveWide: 1 if the read it stopped on was already a normal line, else 0 */
    function leftOnLine(): number {
        const on = countOn()
        return on >= 1 && on <= 3 ? 1 : 0
    }

    /** follow the line and stop on the n-th crossing */
    /** base speed for this moment: fast on segments that are only passed, braking on the last one */
    function segmentBase(c: number, n: number, lens: number[], segT: number, endBase: number): number {
        const fast = fastSpeed > runMin && c < lens.length
        if (c < n - 1) return fast ? fastSpeed : runMin
        if (!fast) return endBase
        const boostMs = (lens[c] - brakeCm) / cmPerMs
        return control.millis() - segT < boostMs ? fastSpeed : endBase
    }

    function rampTo(cur: number, target: number): number {
        return target <= cur ? target : Math.min(target, cur + RAMP)
    }

    function follow(n: number, lens: number[], stopAtEnd: boolean, endBase: number): boolean {
        if (!leaveWide()) return false
        prevError = 0
        let c = 0
        let clear = CLEAR_READS   // armed at the start; only a crossing just counted disarms it
        const t = control.millis()
        let segT = t
        let base = runMin
        let fourReads = 0   // a crossing crossed at an angle (fast) may only ever light 4 sensors
        while (true) {
            readSensors()
            const on = countOn()
            fourReads = on >= 4 ? fourReads + 1 : 0
            if ((isWide() || fourReads >= 2) && clear >= CLEAR_READS) {
                // learn the real speed on a segment driven fast from end to end (not the first: it starts from rest)
                if (fastSpeed > runMin && c >= 1 && c < n - 1 && c < lens.length) {
                    // cm per ms; a real robot runs about 0.03–0.15 (30–150 cm/s)
                    const v = Math.max(0.02, Math.min(0.3, lens[c] / Math.max(1, control.millis() - segT)))
                    cmPerMs = speedLearned ? (cmPerMs + v) / 2 : v
                    speedLearned = true
                }
                c++
                if (c >= n) {
                    if (stopAtEnd) PTKidsBITRobot.motorStop()
                    else PTKidsBITRobot.motorGo(moveSpeed(), moveSpeed())   // roll on into the turn
                    return true
                }
                if (!leaveWideAt(base)) return false
                clear = leftOnLine()
                segT = control.millis()
            } else {
                if (on >= 1 && on <= 3) clear++   // only a counted crossing resets it
                base = rampTo(base, segmentBase(c, n, lens, segT, endBase))
                pdStepAt(base, Math.max(runMax, base))
            }
            if (stopRequested || control.millis() - t > FOLLOW_TIMEOUT) return false
        }
    }

    /** spin (dir 0 = left, 1 = right) blind for blindMs, then until a centre sensor sees a line */
    //% blockHidden=true
    export function spinToLine(dir: number, speed: number, blindMs: number): boolean { return spin(dir, speed, blindMs, true) }

    function spin(dir: number, speed: number, blindMs: number, stopAtEnd: boolean): boolean {
        const l = dir == 0 ? -speed : speed
        PTKidsBITRobot.motorGo(l, -l)
        const tb = control.millis()
        if (pendingArm) {
            // lower the arm while already spinning (the servo write itself takes ~100 ms on board V1)
            pendingArm = false
            servoNow(armServo, armDown)
        }
        const left = blindMs - (control.millis() - tb)
        if (left > 0) basic.pause(left)
        const t = control.millis()
        // phase 1: leave the line the centre sensors may still be on
        while (true) {
            readSensors()
            if (!centerOnLine()) break
            PTKidsBITRobot.motorGo(l, -l)
            if (stopRequested || control.millis() - t > SPIN_TIMEOUT) return false
        }
        // phase 2: stop on the next line
        while (true) {
            readSensors()
            position()   // keep "which side the line was last seen" fresh for the line follower
            if (centerOnLine()) {
                if (stopAtEnd) PTKidsBITRobot.motorStop()
                return true
            }
            PTKidsBITRobot.motorGo(l, -l)
            if (stopRequested || control.millis() - t > SPIN_TIMEOUT) return false
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
            while (!sideOnLine() && !pollStop()) {
                PTKidsBITRobot.motorGo(moveSpeed(), moveSpeed())
                if (control.millis() - t > 1000) break   // missed it: turn from here rather than run away
            }
            if (stopRequested) return false
            if (smoothTurn) roll(sideFwdMs)
            else { PTKidsBITRobot.motorStop(); nudge(sideFwdMs) }
        } else if (smoothTurn) roll(turnFwdMs)
        else nudge(turnFwdMs)
        return spin(dir, turnSpeed, turnBlindMs, !smoothTurn)
    }

    // last angle written to each servo by this extension (-1 = unknown): identical writes are skipped
    let lastArm = -1
    let lastGrip = -1

    /** move a servo without waiting (the board V1 servo block waits 100 ms every time) */
    function servoNow(sv: Servo_Write, deg: number): void {
        const isArm = sv == armServo
        if (isArm ? lastArm == deg : lastGrip == deg) return
        PTKidsBITRobot.servoWriteNoWait(sv, deg)
        if (isArm) lastArm = deg
        else lastGrip = deg
    }

    /** speed for timed moves (nudges, roll before a spin, side-sensor search) and the last bit before a crossing */
    function moveSpeed(): number { return crossSpeed > 0 ? crossSpeed : runMin }

    /** keep driving forward ms at the base speed without stopping afterwards */
    function roll(ms: number): void {
        if (ms <= 0) return
        PTKidsBITRobot.motorGo(moveSpeed(), moveSpeed())
        basic.pause(ms)
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
            if (stopRequested || control.millis() - t > 3000) return false
        }
    }

    /**
     * after gripping in "stop on the crossing" mode: the robot crept forward past the crossing.
     * Reverse until the front sensors are on that crossing again, so the side sensors are behind it
     * for the turn (a fixed-time reverse can leave them past the line after the stop overshoot).
     */
    function backOntoBoxCrossing(): boolean {
        // always: the robot slides past the crossing when it stops, whatever the Box time is.
        // Reversing a little too far is harmless (the turn drives forward to the line again).
        const v = Math.min(runMin, approachSpeed)
        if (sideTurn && sideCalibrated()) {
            // the side sensors decide where the turn happens: after gripping they are usually past the
            // crossing line. Reverse until one of them is on it again (front sensors can't tell the
            // crossing from the box square's edge). Short cap: never far enough to reach the next crossing.
            const t0 = control.millis()
            const cap = Math.max(0, boxMsFor(curNode)) * moveSpeed() / v * 1.5 + 250
            while (!sideOnLine() && !pollStop()) {
                PTKidsBITRobot.motorGo(-v, -v)
                if (control.millis() - t0 > cap) break
            }
            PTKidsBITRobot.motorStop()
            return !stopRequested
        }
        // already on the crossing (small Box time): stay. Reversing from here would reach the NEXT crossing.
        readSensors()
        if (countOn() >= 4) { PTKidsBITRobot.motorStop(); return true }
        // short cap: the crossing is only a few cm behind; never far enough to reach the next one
        const limit = Math.max(0, boxMsFor(curNode)) * moveSpeed() / v * 1.5 + 250
        const t = control.millis()
        let lineReads = 0
        while (true) {
            readSensors()
            const on = countOn()
            // the stub line between box and crossing (not white: the inside of the box square is white too)
            if (on >= 1 && on <= 3) lineReads++
            else if (on >= 4 && lineReads >= 2) break        // 4+ sensors after the stub line: the crossing
            PTKidsBITRobot.motorGo(-v, -v)
            if (stopRequested) break
            if (control.millis() - t > limit) break          // not found: turn from here
        }
        PTKidsBITRobot.motorStop()
        return !stopRequested
    }

    /**
     * reverse out of START / a zone: the frame edge is ignored until the plain line has been seen
     * for a few reads, then stop on the nearest crossing.
     */
    function backOutOfZone(stopOnSide: boolean): boolean {
        const t = control.millis()
        // reverse slowly by default (easier to keep the line); setZoneBackSpeed overrides it
        const v = zoneBackSpeed > 0 ? zoneBackSpeed : Math.min(runMin, approachSpeed)
        let lineReads = 0
        let tLine = t
        prevError = 0
        while (true) {
            readSensors()
            const on = countOn()
            // the plain line seen at least 3 times in total (a short drift off it does not reset this)
            if (on >= 1 && on <= 3) { lineReads++; tLine = control.millis() }
            // the crossing in front of the zone: 4 sensors are enough (reversing at a slight angle)
            if (on >= 4 && lineReads >= 3) { PTKidsBITRobot.motorStop(); return true }
            // a turn follows: the side sensors (behind the front ones) reach the crossing first —
            // stop there, so the turn needs no drive forward to find the line again
            if (stopOnSide && lineReads >= 3 && sideOnLine()) { PTKidsBITRobot.motorStop(); return true }
            // line lost after it was found: the crossing was missed, stop before running far off
            if (on == 0 && lineReads >= 3 && control.millis() - tLine > 300) { PTKidsBITRobot.motorStop(); return false }
            if (on >= 1 && on <= 3) {
                // steer while reversing: same turn direction as forward following, half the gain
                let pd = runKP * 0.5 * (position() - 1500)
                const cap = v * 0.8                       // both wheels keep reversing
                if (pd > cap) pd = cap
                else if (pd < -cap) pd = -cap
                PTKidsBITRobot.motorGo(-v - pd, -v + pd)
            } else PTKidsBITRobot.motorGo(-v, -v)
            if (stopRequested || control.millis() - t > 4000) return false
        }
    }

    function uturn(kind: number): boolean {
        // START / zone: come back to the frame edge first, then spin like on a crossing
        if (kind == MissionNav.K_ZONE && !backToCrossing(0)) return false
        return spin(1, uturnSpeed, uturnBlindMs, true)
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
            if (stopRequested || control.millis() - t > 2000) return false
        }
    }

    function arrive(arg: number, lens: number[]): boolean {
        const kd = arg % 10
        const pass = Math.idiv(arg, 10)
        if (kd == MissionNav.K_YELLOW) {
            // stopped on the stop bar; the yellow box sits just past it (no line)
            nudge(arriveYellowMs)
            return true
        }
        const toBox = kd == MissionNav.K_BOX
        const slow = Math.min(approachSpeed, runMin)
        if (toBox && boxStopAtCrossing) {
            // stop on the crossing right before the box: drive there like any stop crossing,
            // fast where possible, braking to the approach speed on the last segment
            if (pass > 0 && !follow(pass, lens, true, slow)) return false
            PTKidsBITRobot.motorStop()
            showWhy(0)
            nudge(boxMsFor(goalNode))
            return true
        }
        // into a box with nothing to pass: slow from the start
        if (!leaveWideAt(toBox && pass == 0 ? slow : runMin)) return false
        prevError = 0
        let c = 0
        let seenLine = false
        let lostReads = 0
        const t = control.millis()
        let tLast = t        // when the robot left the last crossing before the box
        let base = runMin
        let clear = CLEAR_READS   // armed at the start; only a crossing just counted disarms it
        while (true) {
            readSensors()
            const on = countOn()
            if (centerOnLine()) seenLine = true
            lostReads = on == 0 ? lostReads + 1 : 0
            const ready = clear >= CLEAR_READS
            if (on >= 1 && on <= 3) clear++   // only a counted crossing resets it
            // into a box: slow down from the second-last crossing; after the last one the small
            // box frame (≥ 4 sensors) or the approach timer ends the run
            const slowRun = toBox && c >= pass - 1
            const boxRun = toBox && c >= pass
            if (on >= 5 && c < pass && ready) {
                c++
                if (!leaveWideAt(slowRun ? slow : Math.max(runMin, base))) return false
                clear = leftOnLine()
                if (c >= pass) tLast = control.millis()
            } else {
                // why we stop: 0 frame edge, 1 end of the line, 2 approach timer (only after the last crossing:
                // there is no line end between crossings, a lost line there is just a wobble)
                const pastLast = c >= pass
                let why = -1
                if (ready && (on >= 5 || (boxRun && on >= 4))) why = 0
                else if (pastLast && seenLine && lostReads >= 3) why = 1
                else if (boxRun && approachStopMs > 0 && control.millis() - tLast >= approachStopMs) why = 2
                if (why >= 0) {
                    PTKidsBITRobot.motorStop()
                    showWhy(why)
                    break
                }
                if (slowRun) pdStepAt(slow, runMax)
                else {
                    // segments that end in a crossing to pass (c < pass) can run fast
                    base = rampTo(base, fastSpeed > runMin && c < pass && !(toBox && c >= pass - 1) ? fastSpeed : runMin)
                    pdStepAt(base, Math.max(runMax, base))
                }
            }
            if (stopRequested || control.millis() - t > FOLLOW_TIMEOUT) return false
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
        const v = ms > 0 ? moveSpeed() : -moveSpeed()
        PTKidsBITRobot.motorGo(v, v)
        basic.pause(Math.abs(ms))
        PTKidsBITRobot.motorStop()
    }

    /**
     * turn towards a box that is right after this crossing: the open gripper would sweep into it.
     * lift + close, turn, back the sensors onto the crossing, then lower + open before driving in.
     */
    function turnIntoBox(dir: number): boolean {
        servoNow(armServo, armUp)
        servoNow(gripServo, gripClose)
        basic.pause(SERVO_MS)
        if (!turn(dir)) return false
        const ok = backToCrossing(0)
        servoNow(armServo, armDown)
        servoNow(gripServo, gripOpen)
        basic.pause(SERVO_MS)
        return ok
    }

    /** index of the last FOLLOW before arriving at the white/black zone (-1: none / other goal) */
    function lastFollowBeforeZone(acts: number[]): number {
        const n = acts.length
        if (!lowerArmBeforeZone || n < 2) return -1
        if (acts[n - 2] != MissionNav.OP_ARRIVE || acts[n - 1] % 10 != MissionNav.K_ZONE) return -1
        if (goalNode != MissionNav.WZ && goalNode != MissionNav.BZ) return -1
        for (let i = n - 4; i >= 0; i -= 2) if (acts[i] == MissionNav.OP_FOLLOW) return i
        return -1
    }

    function run(acts: number[]): boolean {
        const armAt = lastFollowBeforeZone(acts)
        let tn = curNode, th = curHeading   // where the plan says the robot is (for segment lengths)
        for (let i = 0; i < acts.length; i += 2) {
            const op = acts[i], arg = acts[i + 1]
            let ok = true
            let lens: number[] = []
            if (op == MissionNav.OP_FOLLOW || (op == MissionNav.OP_ARRIVE && arg % 10 != MissionNav.K_YELLOW)) {
                const k = op == MissionNav.OP_FOLLOW ? arg : Math.idiv(arg, 10)
                lens = MissionNav.segmentLengths(tn, th, k)
                tn = MissionNav.segmentEnd()
            } else if (op == MissionNav.OP_BACK) tn = MissionNav.neighbor(tn, (th + 2) % 4)
            else if (op == MissionNav.OP_UTURN) th = (th + 2) % 4
            else if (op == MissionNav.OP_TURN) th = (th + (arg == 1 ? 1 : 3)) % 4
            const nextIsBox = i + 3 < acts.length && acts[i + 2] == MissionNav.OP_ARRIVE && acts[i + 3] == MissionNav.K_BOX
            if (op == MissionNav.OP_TURN && gripOn && nextIsBox) ok = turnIntoBox(arg)
            else if (op == MissionNav.OP_UTURN) ok = uturn(arg)
            else if (op == MissionNav.OP_EXIT) ok = exitZone()
            else if (op == MissionNav.OP_FOLLOW) ok = follow(arg, lens, !(smoothTurn && i + 2 < acts.length && acts[i + 2] == MissionNav.OP_TURN), moveSpeed())
            else if (op == MissionNav.OP_TURN) ok = turn(arg)
            else if (op == MissionNav.OP_ARRIVE) {
                if (lowerArmBeforeBox && (arg % 10 == MissionNav.K_BOX || arg % 10 == MissionNav.K_YELLOW)) {
                    // arm down for the last stretch only: the robot is still a crossing away from the box
                    servoNow(gripServo, gripOpen)
                    servoNow(armServo, armDown)
                    if (Math.idiv(arg, 10) == 0) basic.pause(150)   // already next to the box: let the arm come down
                }
                ok = arrive(arg, lens)
            }
            else if (op == MissionNav.OP_BACK && arg != MissionNav.K_ZONE && boxStopAtCrossing && MissionNav.kind(curNode) == MissionNav.K_BOX) ok = backOntoBoxCrossing()
            else if (op == MissionNav.OP_BACK) ok = arg == MissionNav.K_ZONE ? backOutOfZone(sideTurn && sideCalibrated() && i + 2 < acts.length && acts[i + 2] == MissionNav.OP_TURN) : backToCrossing(uturnBackMs)
            if (!ok || stopRequested) { pendingArm = false; return false }
            if (i == armAt) {
                // on the crossing just before the colour zone: lower the box during the turn into it
                if (i + 2 < acts.length && acts[i + 2] == MissionNav.OP_TURN) pendingArm = true
                else servoNow(armServo, armDown)
            }
        }
        if (pendingArm) { pendingArm = false; servoNow(armServo, armDown) }
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
        goToOk(to)
    }

    function goToOk(to: MissionPoint): boolean {
        if (Calibrating) return false   // a button press during SensorCalibrate must not drive the robot
        if (!calibrated()) { failNotCalibrated(); return false }
        const goal = MissionNav.pointNode(to)
        const acts = MissionNav.plan(curNode, curHeading, goal)
        if (acts.length == 0) return curNode == goal
        goalNode = goal
        if (!missionRunning) { stopRequested = false; swallowUndo = false }
        moving = true
        const ok = run(acts)
        moving = false
        if (ok) {
            curNode = goal
            curHeading = MissionNav.finalHeading()
            return true
        }
        if (stopRequested) {
            PTKidsBITRobot.motorStop()
            basic.showIcon(IconNames.Square)
            music.playTone(523, 150)
            return false
        }
        fail()
        return false
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
    //% min.defl=60 max.defl=100 kp.defl=0.046 kd.defl=0.25
    //% min.min=0 min.max=100 max.min=0 max.max=100
    //% group="ตั้งค่า" weight=60 inlineInputMode=inline
    export function setRun(min: number, max: number, kp: number, kd: number): void {
        runMin = min; runMax = max; runKP = kp; runKD = kd
    }

    /**
     * ค่าเลี้ยว 90° ที่แยก
     */
    //% block="ตั้งค่าการเลี้ยว ความเร็ว $speed เวลาหมุนเร็ว $blindMs ms เดินหน้าก่อนเลี้ยว $fwdMs ms"
    //% speed.defl=60 blindMs.defl=125 fwdMs.defl=120
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
    //% speed.defl=60 blindMs.defl=290 backMs.defl=150
    //% speed.min=0 speed.max=100
    //% group="ตั้งค่า" weight=40 inlineInputMode=inline
    export function setUturn(speed: number, blindMs: number, backMs: number): void {
        uturnSpeed = speed; uturnBlindMs = blindMs; uturnBackMs = backMs
    }

    /**
     * เวลาเดินต่อหลังหยุดที่ปลายทาง (ค่าติดลบ = ถอยกลับ)
     */
    //% block="ตั้งค่าการเข้าจุดปลายทาง Box $boxMs ms START $startMs ms Box เหลือง $yellowMs ms"
    //% boxMs.defl=40 startMs.defl=250 yellowMs.defl=50
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
    //% arm.defl=Servo_Write.P8 up.defl=24 down.defl=110 grip.defl=Servo_Write.P12 open.defl=180 close.defl=0
    //% up.min=0 up.max=180 down.min=0 down.max=180 open.min=0 open.max=180 close.min=0 close.max=180
    //% group="ภารกิจอัตโนมัติ" weight=10 inlineInputMode=inline
    //% up.min=0 up.max=180 down.min=0 down.max=180 open.min=0 open.max=180 close.min=0 close.max=180
    //% arm.defl=Servo_Write.P8 grip.defl=Servo_Write.P12 up.defl=24 down.defl=110 open.defl=180 close.defl=0
    export function setGripper(arm: Servo_Write, up: number, down: number, grip: Servo_Write, open: number, close: number): void {
        armServo = arm; armUp = up; armDown = down
        lastArm = -1; lastGrip = -1
        gripServo = grip; gripOpen = open; gripClose = close
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
    //% speed.defl=40 speed.min=10 speed.max=100
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
     * คืนค่าตั้งทั้งหมดเป็นชุดมาตรฐานสำหรับแข่ง (ชุดที่จูนแล้ว ทำภารกิจได้ราว 1:20–1:22)
     * ค่าเหล่านี้ตั้งไว้แล้วตั้งแต่เปิดเครื่อง — ใช้บล็อคนี้เมื่อปรับค่าไปแล้วอยากกลับมาเริ่มใหม่
     */
    //% block="คืนค่าตั้งเริ่มต้นสำหรับแข่ง"
    //% group="ตั้งค่า" weight=110
    export function resetSettings(): void {
        runMin = 60; runMax = 100; runKP = 0.046; runKD = 0.25
        retryBackOne = true
        crossSpeed = 50; zoneBackSpeed = 65
        fastSpeed = 100; brakeCm = 14
        turnSpeed = 60; turnBlindMs = 125; turnFwdMs = 120
        sideTurn = true; sideFwdMs = 100
        uturnSpeed = 60; uturnBlindMs = 290; uturnBackMs = 150
        approachSpeed = 40; approachStopMs = 0
        arriveBoxMs = 40; arriveStartMs = 250; arriveYellowMs = 50; arriveZoneMs = 0
        for (let i = 0; i < 8; i++) { boxArriveMs[i] = 0; boxArriveSet[i] = false }
        boxArriveSet[4] = true; boxArriveSet[5] = true          // UR, DR: 0 ms
        armServo = Servo_Write.P8; armUp = 24; armDown = 110
        gripServo = Servo_Write.P12; gripOpen = 180; gripClose = 0
        lastArm = -1; lastGrip = -1
        gripHoldMs = 100; gripReleaseMs = 50
        smoothTurn = true; boxStopAtCrossing = true; lowerArmBeforeZone = true; showStopReason = false
        lowerArmBeforeBox = false
    }

    //% blockHidden=true
    export function settingsForTest(): number[] {
        const b = (x: boolean) => x ? 1 : 0
        return [runMin, runMax, runKP, runKD, crossSpeed, zoneBackSpeed, fastSpeed, brakeCm,
            turnSpeed, turnBlindMs, turnFwdMs, b(sideTurn), sideFwdMs, uturnSpeed, uturnBlindMs, uturnBackMs,
            approachSpeed, arriveBoxMs, arriveStartMs, arriveYellowMs, arriveZoneMs,
            armUp, armDown, gripOpen, gripClose, gripHoldMs, gripReleaseMs,
            b(smoothTurn), b(boxStopAtCrossing), b(lowerArmBeforeZone), b(showStopReason),
            boxArriveSet[4] ? boxArriveMs[4] : -1, boxArriveSet[5] ? boxArriveMs[5] : -1]
    }

    /** the values the off-robot tests were written for (before the competition set) */
    //% blockHidden=true
    export function useLegacyDefaultsForTest(): void {
        runMin = 30; runMax = 60; runKP = 0.05; runKD = 0.1
        retryBackOne = false
        crossSpeed = 0; zoneBackSpeed = 0; fastSpeed = 0; brakeCm = 10
        turnSpeed = 50; turnBlindMs = 150; turnFwdMs = 120; sideTurn = true; sideFwdMs = 100
        uturnSpeed = 50; uturnBlindMs = 350; uturnBackMs = 150
        approachSpeed = 30; approachStopMs = 0
        arriveBoxMs = 0; arriveStartMs = 300; arriveYellowMs = 250; arriveZoneMs = 0
        for (let i = 0; i < 8; i++) { boxArriveMs[i] = 0; boxArriveSet[i] = false }
        armServo = Servo_Write.P8; armUp = 90; armDown = 0; gripServo = Servo_Write.P12; gripOpen = 180; gripClose = 0
        lastArm = -1; lastGrip = -1
        gripHoldMs = 250; gripReleaseMs = 200
        smoothTurn = false; boxStopAtCrossing = false; lowerArmBeforeZone = true; showStopReason = false
    }

    /**
     * ความเร็วเข้าแยก: ใช้ตอนเบรกเข้าแยกที่ต้องหยุด/เลี้ยว ตอนเซนเซอร์ข้างหาแยก ตอนเดินต่อก่อนหมุน
     * และตอนเดินหน้า/ถอยตามเวลา (เข้ากล่อง, START, กล่องเหลือง) — ระยะเลี้ยวจึงไม่เปลี่ยนเมื่อเพิ่ม RUN_MIN
     * 0 = ใช้ความเร็วต่ำสุดของตั้งค่าการวิ่ง (แบบเดิม)
     */
    //% block="ตั้งค่าความเร็วเข้าแยก $speed"
    //% speed.defl=50
    //% group="ตั้งค่า" weight=100
    export function setCrossingSpeed(speed: number): void {
        crossSpeed = speed
    }

    /**
     * ความเร็วตอนถอยออกจากช่องขาว/ช่องดำ (ถอยตามเส้นจนถึงแยกหน้าช่อง)
     * 0 = ใช้ค่าที่น้อยกว่าระหว่างความเร็วต่ำสุดกับความเร็วเข้า Box (แบบเดิม)
     */
    //% block="ตั้งค่าความเร็วถอยออกจากช่อง $speed"
    //% speed.defl=65
    //% group="ตั้งค่า" weight=96
    export function setZoneBackSpeed(speed: number): void {
        zoneBackSpeed = speed
    }

    /**
     * วิ่งเร็วบนทางตรง: ช่วงที่แค่วิ่งผ่านแยกใช้ความเร็วนี้ตลอด ช่วงสุดท้ายก่อนแยกที่ต้องหยุด/เลี้ยว
     * ชะลอลงเป็นความเร็วต่ำสุดเมื่อเหลือระยะเบรก (หุ่นวัดความเร็วจริงจากเวลาวิ่งระหว่างแยกเอง)
     * ความเร็ว 0 = ปิด · ควรตั้ง "สูงสุด" ใน ตั้งค่าการวิ่ง ให้มากกว่าความเร็วนี้ เพื่อให้มีแรงแก้เส้น
     */
    //% block="ตั้งค่าวิ่งเร็วทางตรง ความเร็ว $speed เบรกก่อนถึงแยก $cm ซม."
    //% speed.defl=100 cm.defl=14
    //% group="ตั้งค่า" weight=99
    export function setFastRun(speed: number, cm: number): void {
        fastSpeed = speed
        brakeCm = cm
    }

    /**
     * เลี้ยวแบบไม่หยุด: วิ่งเข้าแยกแล้วหมุนต่อทันที หมุนเจอเส้นแล้ววิ่งตามเส้นต่อเลย
     * (ปิด = หยุดบนแยก หยุดก่อนหมุน และหยุดหลังหมุน แบบเดิม)
     */
    //% block="เลี้ยวแบบไม่หยุด $on"
    //% on.defl=true
    //% group="ตั้งค่า" weight=97
    export function setSmoothTurn(on: boolean): void {
        smoothTurn = on
    }

    /** ความเร็วจริงที่หุ่นวัดได้ตอนวิ่งเร็ว (ซม./วินาที) */
    //% block="ความเร็ววิ่งเร็วที่วัดได้ (ซม./วินาที)"
    //% group="ตั้งค่า" weight=98
    export function learnedSpeed(): number {
        return Math.round(cmPerMs * 1000)
    }

    //% blockHidden=true
    export function resetLearnedSpeed(): void {
        cmPerMs = CMPMS_START
        speedLearned = false
    }

    /**
     * เวลารอของกริปเปอร์ในภารกิจอัตโนมัติ: หลังหนีบก่อนออกวิ่ง และหลังอ้าวางก่อนถอย (ms)
     */
    //% block="ตั้งค่าเวลากริปเปอร์ หนีบ $holdMs ms ปล่อย $releaseMs ms"
    //% holdMs.defl=100 releaseMs.defl=50
    //% group="ภารกิจอัตโนมัติ" weight=9
    export function setGripTiming(holdMs: number, releaseMs: number): void {
        gripHoldMs = holdMs
        gripReleaseMs = releaseMs
    }

    /**
     * ระยะเดินหน้าเข้ากล่องเฉพาะจุด (ใช้กับ "หยุดที่แยกก่อนถึง Box") แทนค่า Box ของ ตั้งค่าการเข้าจุดปลายทาง
     * UL/DL อยู่ห่างแยกราว 10 ซม. ส่วน UR/DR/T1/T2/B1/B2 ราว 8 ซม. จึงควรใช้เวลาน้อยกว่า
     */
    //% block="ตั้งค่าเดินหน้าเข้ากล่องที่ $spot $ms ms"
    //% spot.defl=MissionPoint.UR ms.defl=0
    //% group="ตั้งค่า" weight=27
    export function setBoxArrive(spot: MissionPoint, ms: number): void {
        const i = spot - MissionPoint.UL
        if (i < 0 || i >= 8) return
        boxArriveMs[i] = ms
        boxArriveSet[i] = true
    }

    /** ให้ทุกกล่องกลับไปใช้ค่า Box เดียวกัน */
    //% blockHidden=true
    export function clearBoxArrive(): void {
        for (let i = 0; i < 8; i++) boxArriveSet[i] = false
    }

    /**
     * หยุดที่แยกสุดท้ายก่อนถึง Box แทนการวิ่งไปหาขอบกรอบ Box
     * ใช้เมื่อกริปเปอร์ยื่นไปข้างหน้าไกล (วิ่งถึงขอบกรอบแล้วกริปเปอร์ชนกล่อง)
     * ปรับระยะต่อด้วย "ตั้งค่าการเข้าจุดปลายทาง Box … ms" (บวก = เดินหน้าต่อ, ลบ = ถอย)
     */
    //% block="หยุดที่แยกก่อนถึง Box $on"
    //% on.defl=true
    //% group="ตั้งค่า" weight=28
    export function setBoxStopAtCrossing(on: boolean): void {
        boxStopAtCrossing = on
    }

    /**
     * แสดงเหตุผลที่หุ่นหยุดตอนถึงปลายทาง (ไฟแถวบนสุด): ซ้าย = เจอขอบกรอบ, กลาง = สุดปลายเส้น, ขวา = หมดเวลาเข้า Box
     */
    //% block="แสดงเหตุผลการหยุด $on"
    //% on.defl=false
    //% group="ตั้งค่า" weight=2
    export function setShowStopReason(on: boolean): void {
        showStopReason = on
    }

    // ================= ภารกิจอัตโนมัติ (auto mission) =================

    // box index i: 0 UL, 1 DL, 2 T1, 3 B1, 4 UR, 5 DR, 6 T2, 7 B2
    const BOX_POINT = [MissionPoint.UL, MissionPoint.DL, MissionPoint.T1, MissionPoint.B1, MissionPoint.UR, MissionPoint.DR, MissionPoint.T2, MissionPoint.B2]
    // LED of each box spot on the 5x5 screen (top row = white-zone side of the field)
    const BOX_LED_X = [0, 0, 1, 1, 4, 4, 3, 3]
    const BOX_LED_Y = [1, 3, 0, 4, 1, 3, 0, 4]
    // button input goes round the screen clockwise: UL, T1, T2, UR, DR, B2, B1, DL
    const INPUT_ORDER = [0, 2, 6, 4, 5, 7, 3, 1]
    let boxColor = [-1, -1, -1, -1, -1, -1, -1, -1]
    let doneMask = 0
    let testDrive: (p: number) => boolean = (p: number) => true
    let useTestDrive = false

    function boxIndex(spot: MissionPoint): number { return BOX_POINT.indexOf(spot) }

    function drive(p: MissionPoint): boolean {
        if (useTestDrive) return testDrive(p)
        return goToOk(p)
    }

    function servo(s: Servo_Write, d: number): void { servoNow(s, d) }

    /** arm down + gripper open, ready to drive onto a box (moves while the robot drives there) */
    function gripReady(): void {
        servo(armServo, armDown)
        servo(gripServo, gripOpen)
    }
    /** close on the box, then lift: the arm rises while the robot already backs away */
    function gripTake(): void {
        servo(gripServo, gripClose)
        basic.pause(gripHoldMs)
        servo(armServo, armUp)
    }
    /** put the box down and let go; the empty arm lifts while the robot backs out */
    function gripDrop(): void {
        servo(armServo, armDown)                       // already down when lowered on the last crossing
        if (!lowerArmBeforeZone) basic.pause(SERVO_MS)
        servo(gripServo, gripOpen)
        basic.pause(gripReleaseMs)
        servo(armServo, armUp)
    }

    function colorsReady(): boolean {
        let black = 0, white = 0
        for (let i = 0; i < 8; i++) {
            if (boxColor[i] == 0) black++
            else if (boxColor[i] == 1) white++
        }
        return black == 4 && white == 4
    }

    function showColors(): void {
        basic.clearScreen()
        for (let i = 0; i < 8; i++) if (boxColor[i] == 0) led.plot(BOX_LED_X[i], BOX_LED_Y[i])
    }

    /** 0 = A, 1 = B; waits until the button is released */
    function waitButton(blinkX: number, blinkY: number): number {
        let on = false
        let t = control.millis()
        while (true) {
            if (input.buttonIsPressed(Button.A)) {
                while (input.buttonIsPressed(Button.A)) basic.pause(20)
                return 0
            }
            if (input.buttonIsPressed(Button.B)) {
                while (input.buttonIsPressed(Button.B)) basic.pause(20)
                return 1
            }
            if (control.millis() - t >= 200) {
                on = !on
                if (on) led.plot(blinkX, blinkY)
                else led.unplot(blinkX, blinkY)
                t = control.millis()
            }
            basic.pause(20)
        }
    }

    /** forward time for a box node (its own value if set, else the common Box time) */
    function boxMsFor(node: number): number {
        const i = node - MissionNav.UL
        if (i >= 0 && i < 8 && boxArriveSet[i]) return boxArriveMs[i]
        return arriveBoxMs
    }

    /** short rising tune: colours entered, ready to start */
    function playDoneTune(): void {
        const notes = [523, 659, 784, 1047]   // C5 E5 G5 C6
        for (let i = 0; i < notes.length; i++) music.playTone(notes[i], i == notes.length - 1 ? 300 : 120)
    }

    /**
     * ใส่สีกล่องด้วยปุ่มบนหุ่น ไฟกะพริบ = ช่องที่กำลังถาม (แถวบน = ฝั่งช่องขาว)
     * กด A = กล่องดำ, กด B = กล่องขาว วนตามเข็มนาฬิกา 8 ช่อง แล้วแสดงสรุป (ไฟติด = ดำ)
     * ถ้าไม่ใช่ดำ 4 ขาว 4 จะแสดง X แล้วให้ใส่ใหม่
     */
    //% block="ใส่สีกล่องด้วยปุ่ม A=ดำ B=ขาว"
    //% group="ภารกิจอัตโนมัติ" weight=100
    export function inputBoxColors(): void {
        while (true) {
            for (let i = 0; i < 8; i++) boxColor[i] = -1
            basic.clearScreen()
            for (let k = 0; k < 8; k++) {
                const i = INPUT_ORDER[k]
                boxColor[i] = waitButton(BOX_LED_X[i], BOX_LED_Y[i])
                if (boxColor[i] == 0) led.plot(BOX_LED_X[i], BOX_LED_Y[i])
                else led.unplot(BOX_LED_X[i], BOX_LED_Y[i])
                music.playTone(boxColor[i] == 0 ? 523 : 784, 80)
            }
            if (colorsReady()) {
                showColors()
                // work out the box order now (slow the first time), so pressing B drives off at once
                MissionNav.bestOrder(boxColor.slice(), 0)
                playDoneTune()
                basic.pause(1000)
                return
            }
            basic.showIcon(IconNames.No)
            music.playTone(262, 300)
            basic.pause(800)
        }
    }

    /**
     * กำหนดสีกล่องของจุดหนึ่งด้วยบล็อค (ใช้แทนการกดปุ่มได้)
     */
    //% block="ตั้งสีกล่องที่ $spot เป็น $color"
    //% spot.defl=MissionPoint.UL
    //% group="ภารกิจอัตโนมัติ" weight=95
    export function setBoxColor(spot: MissionPoint, color: BoxColor): void {
        const i = boxIndex(spot)
        if (i >= 0) boxColor[i] = color
    }

    /**
     * เริ่มภารกิจอัตโนมัติจาก START: หุ่นคำนวณลำดับที่เร็วที่สุดเอง
     * หยิบกล่อง → ส่งเข้าช่องสีเดียวกัน ครบ 8 กล่อง แล้วนำกล่องเหลืองกลับ START (คีบค้างไว้ ไม่ปล่อย)
     * ถ้าพลาดกลางทาง หุ่นหยุด: RETRY วางหุ่นที่ START แล้วเรียกบล็อคนี้อีกครั้ง หุ่นทำต่อเฉพาะกล่องที่ยังไม่ส่ง
     */
    //% block="เริ่มภารกิจอัตโนมัติ"
    //% group="ภารกิจอัตโนมัติ" weight=90
    export function startAutoMission(): void {
        if (Calibrating || missionRunning) return
        missionRunning = true; stopRequested = false; swallowUndo = false
        autoMission()
        missionRunning = false
        PTKidsBITRobot.motorStop()
    }

    function autoMission(): void {
        if (!colorsReady()) {
            basic.showIcon(IconNames.No)
            music.playTone(262, 300)
            return
        }
        setPosition(MissionPoint.START)
        lastArm = -1; lastGrip = -1   // the servos may have been moved by other blocks since
        const colors = boxColor.slice()
        const order = MissionNav.bestOrder(colors, doneMask)
        for (let k = 0; k < order.length; k++) {
            const i = order[k]
            MissionNav.setPresentMask(255 & ~doneMask)
            gripReady()
            if (!drive(BOX_POINT[i])) { MissionNav.setPresentMask(0); return }
            gripTake()
            MissionNav.setPresentMask(255 & ~doneMask & ~(1 << i))
            if (!drive(colors[i] == 1 ? MissionPoint.WhiteZone : MissionPoint.BlackZone)) { MissionNav.setPresentMask(0); return }
            gripDrop()
            doneMask |= 1 << i
        }
        MissionNav.setPresentMask(0)
        gripReady()
        if (!drive(MissionPoint.Yellow)) return
        gripTake()
        if (!drive(MissionPoint.START)) return
        // finish holding the yellow box in START (no release)
        basic.showIcon(IconNames.Yes)
        music.playTone(988, 400)
    }

    /**
     * เริ่มภารกิจใหม่ทั้งหมด (ลืมว่ากล่องไหนส่งแล้ว) สีกล่องยังจำไว้
     */
    //% block="เริ่มภารกิจอัตโนมัติใหม่ทั้งหมด"
    //% group="ภารกิจอัตโนมัติ" weight=80
    export function resetAutoMission(): void {
        doneMask = 0
    }

    /**
     * บอกหุ่นว่ากล่องที่จุดนี้ส่งแล้ว / ไม่ต้องเก็บ
     */
    //% block="ข้ามกล่องที่ $spot"
    //% spot.defl=MissionPoint.UL
    //% group="ภารกิจอัตโนมัติ" weight=70
    export function skipBox(spot: MissionPoint): void {
        const i = boxIndex(spot)
        if (i >= 0) doneMask |= 1 << i
    }

    /**
     * จำนวนกล่องที่ส่งเข้าช่องแล้ว
     */
    //% block="จำนวนกล่องที่ส่งแล้ว"
    //% group="ภารกิจอัตโนมัติ" weight=60
    export function boxesDone(): number {
        let n = 0
        for (let i = 0; i < 8; i++) if (doneMask & (1 << i)) n++
        return n
    }

    /**
     * ลดแขนลงตอนหุ่นหยุดที่แยกสุดท้ายก่อนเข้าช่องขาว/ช่องดำ (เปิดอยู่เป็นค่าเริ่มต้น)
     * ถึงช่องแล้วอ้ามือวางกล่องได้ทันที
     */
    //% block="ลดแขนที่แยกก่อนถึงช่องขาว/ช่องดำ $on"
    //% on.defl=true
    //% group="ภารกิจอัตโนมัติ" weight=8
    export function setLowerArmBeforeZone(on: boolean): void {
        lowerArmBeforeZone = on
    }

    // ================= เขียนเส้นทางเอง (manual routes, same behaviour as the auto mission) =================

    /**
     * บอกหุ่นว่ากล่องทั้ง 8 จุดวางอยู่บนสนาม (ใส่ตอนเริ่มรอบ)
     * หุ่นจะเลี่ยงการเลี้ยวข้างกล่องที่ยังไม่เก็บ เหมือนภารกิจอัตโนมัติ · "หยิบกล่อง" จะจำเองว่ากล่องนั้นเก็บแล้ว
     */
    //% block="ตั้งกล่องบนสนามครบ 8 กล่อง"
    //% group="เขียนเส้นทางเอง" weight=100
    export function setAllBoxesOnField(): void {
        MissionNav.setPresentMask(255)
    }

    /**
     * เตรียมหนีบ: อ้ามือและยกแขนขึ้น (ใส่ก่อนออกไปหากล่อง วิ่งปกติแขนยกไว้)
     */
    //% block="เตรียมหนีบ (อ้ามือ ยกแขน)"
    //% group="เขียนเส้นทางเอง" weight=90
    export function prepareGrip(): void {
        servoNow(gripServo, gripOpen)
        servoNow(armServo, armUp)
    }

    /**
     * หยิบกล่อง: ลดแขน (ถ้ายังไม่ลง) หนีบ รอตามเวลาหนีบ แล้วยกแขน · หุ่นจำว่ากล่องจุดนี้ถูกเก็บแล้ว
     */
    //% block="หยิบกล่อง"
    //% group="เขียนเส้นทางเอง" weight=80
    export function pickBox(): void {
        if (lastArm != armDown) {
            servoNow(armServo, armDown)
            basic.pause(SERVO_MS)
        }
        servoNow(gripServo, gripClose)
        basic.pause(gripHoldMs)
        servoNow(armServo, armUp)
        const i = curNode - MissionNav.UL
        if (MissionNav.kind(curNode) == MissionNav.K_BOX && i >= 0 && i < 8) MissionNav.setPresentMask(MissionNav.getPresentMask() & ~(1 << i))
    }

    /**
     * วางกล่อง: ลดแขน (ปกติลงไว้แล้วตั้งแต่ก่อนเข้าช่อง) อ้ามือ รอตามเวลาปล่อย แล้วยกแขน
     */
    //% block="วางกล่อง"
    //% group="เขียนเส้นทางเอง" weight=70
    export function dropBox(): void {
        if (lastArm != armDown) {
            servoNow(armServo, armDown)
            basic.pause(SERVO_MS)
        }
        servoNow(gripServo, gripOpen)
        basic.pause(gripReleaseMs)
        servoNow(armServo, armUp)
    }

    // ---- trip list: the order the kids write, run like the auto mission (stop on a miss, RETRY resumes) ----
    let tripBox: number[] = []     // box node per trip, in the written order
    let tripWhite: boolean[] = []  // true = to the white zone
    let retryBackOne = true        // after a miss / stop, resume one trip earlier
    let tripsDoneCount = 0         // trips delivered; = length + 1 once the yellow box is back at START

    /** ล้างรายการเที่ยวทั้งหมด (ใส่ก่อน "เพิ่มเที่ยว") */
    //% block="ล้างรายการเที่ยว"
    //% group="เขียนเส้นทางเอง" weight=99
    export function clearTrips(): void {
        tripBox = []
        tripWhite = []
        tripsDoneCount = 0
    }

    /**
     * เพิ่มหนึ่งเที่ยวต่อท้ายรายการ: หยิบกล่องที่จุดนี้ แล้วส่งเข้าช่องสีที่เลือก
     * เขียนตามลำดับที่อยากให้หุ่นทำ · กล่องเหลืองหุ่นทำเองเป็นเที่ยวสุดท้ายเสมอ
     */
    //% block="เพิ่มเที่ยว หยิบกล่องที่ $box ส่งไปช่อง $color"
    //% box.defl=MissionPoint.UL
    //% group="เขียนเส้นทางเอง" weight=98
    export function addTrip(box: MissionPoint, color: BoxColor): void {
        const n = MissionNav.pointNode(box)
        if (MissionNav.kind(n) != MissionNav.K_BOX) return
        tripBox.push(n)
        tripWhite.push(color == BoxColor.White)
    }

    /**
     * เริ่มทำเที่ยวตามรายการจาก START: หยิบ → ส่งเข้าช่อง ทีละเที่ยว แล้วคีบกล่องเหลืองกลับ START (ไม่ปล่อย)
     * เลี่ยงกล่องที่ยังวางอยู่เหมือนภารกิจอัตโนมัติ · ถ้าพลาด หุ่นหยุด → RETRY วางที่ START แล้วเรียกบล็อคนี้อีกครั้ง
     * หุ่นทำต่อจากเที่ยวที่พลาด (เริ่มเที่ยวนั้นใหม่ตั้งแต่หยิบ)
     */
    //% block="เริ่ม / ทำต่อ ภารกิจที่เขียนเอง"
    //% group="เขียนเส้นทางเอง" weight=97
    export function runTrips(): void {
        if (Calibrating || missionRunning) return
        missionRunning = true; stopRequested = false; swallowUndo = false
        tripsRun()
        missionRunning = false
        // RETRY: step back one trip, so the trip before the miss is done again too
        const n = tripBox.length
        if (retryBackOne && n > 0 && tripsDoneCount <= n && calibrated()) tripsDoneCount = Math.max(0, tripsDoneCount - 1)
        PTKidsBITRobot.motorStop()
    }

    function tripsRun(): void {
        const n = tripBox.length
        if (n == 0) {
            basic.showIcon(IconNames.No)
            music.playTone(262, 300)
            return
        }
        if (tripsDoneCount > n) return                 // finished: use "เริ่มภารกิจที่เขียนเองใหม่ทั้งหมด" for a new round
        setPosition(MissionPoint.START)
        lastArm = -1; lastGrip = -1                     // the servos may have been moved since
        // boxes still on the field: all but the ones already delivered (a failed trip's box is put back)
        let mask = 255
        for (let k = 0; k < tripsDoneCount; k++) mask &= ~(1 << (tripBox[k] - MissionNav.UL))
        for (let k = tripsDoneCount; k < n; k++) {
            const bit = 1 << (tripBox[k] - MissionNav.UL)
            MissionNav.setPresentMask(mask)
            prepareGrip()
            if (!drive(tripBox[k])) { MissionNav.setPresentMask(0); return }
            pickBox()
            mask &= ~bit
            MissionNav.setPresentMask(mask)
            if (!drive(tripWhite[k] ? MissionPoint.WhiteZone : MissionPoint.BlackZone)) { MissionNav.setPresentMask(0); return }
            dropBox()
            tripsDoneCount = k + 1
        }
        MissionNav.setPresentMask(0)
        prepareGrip()
        if (!drive(MissionPoint.Yellow)) return
        pickBox()
        if (!drive(MissionPoint.START)) return
        tripsDoneCount = n + 1                          // done, holding the yellow box in START
        basic.showIcon(IconNames.Yes)
        music.playTone(988, 400)
    }

    /** เริ่มรอบใหม่: นับเที่ยวใหม่จากเที่ยวแรก (รายการเที่ยวยังอยู่) */
    //% block="เริ่มภารกิจที่เขียนเองใหม่ทั้งหมด"
    //% group="เขียนเส้นทางเอง" weight=96
    export function resetTrips(): void {
        tripsDoneCount = 0
    }

    /** จำนวนเที่ยวที่ส่งกล่องเสร็จแล้ว (ไม่นับกล่องเหลือง) */
    //% block="จำนวนเที่ยวที่ทำแล้ว"
    //% group="เขียนเส้นทางเอง" weight=95
    export function tripsDone(): number {
        return Math.min(tripsDoneCount, tripBox.length)
    }

    /** จริง เมื่อทำครบทุกเที่ยวและคีบกล่องเหลืองกลับถึง START แล้ว */
    //% block="ภารกิจที่เขียนเองเสร็จแล้ว"
    //% group="เขียนเส้นทางเอง" weight=94
    export function tripsFinished(): boolean {
        return tripBox.length > 0 && tripsDoneCount > tripBox.length
    }

    /**
     * ถอยการนับกลับ 1 เที่ยว: ใช้ตอนหุ่นหยุดแล้ว เมื่อเที่ยวล่าสุดวางกล่องไม่สำเร็จจริง (หุ่นคิดว่าสำเร็จ)
     * กด "ทำต่อ" แล้วหุ่นจะทำเที่ยวนั้นใหม่ · การกด A ที่ใช้หยุดหุ่นจะไม่ถูกนับเป็นการถอย
     */
    //% block="ถอยการนับกลับ 1 เที่ยว"
    //% group="เขียนเส้นทางเอง" weight=93
    export function undoTrip(): void {
        if (missionRunning || moving) return
        if (swallowUndo) {
            swallowUndo = false
            if (control.millis() - stopAt < 3000) return
        }
        tripsDoneCount = Math.max(0, Math.min(tripsDoneCount, tripBox.length) - 1)
    }

    /**
     * เมื่อพลาดหรือกดหยุด ให้ย้อนกลับ 1 เที่ยว: กด "ทำต่อ" แล้วหุ่นเริ่มจากเที่ยวก่อนหน้าเที่ยวที่พลาด
     * (วางกล่องของเที่ยวนั้นคืนที่เดิมด้วย) · เปิดไว้ตั้งแต่เริ่ม
     */
    //% block="RETRY ย้อนกลับ 1 เที่ยว $on"
    //% on.defl=true
    //% group="หยุด / ทำต่อ" weight=91
    export function setRetryBackOne(on: boolean): void {
        retryBackOne = on
    }

    /** หยุดหุ่นทันที (ใช้กับปุ่มอื่นได้ เช่น โลโก้) · ปกติปุ่ม A หยุดหุ่นขณะวิ่งให้อยู่แล้ว */
    //% block="หยุดภารกิจทันที"
    //% group="หยุด / ทำต่อ" weight=90
    export function stopMission(): void {
        if (moving || missionRunning) {
            stopRequested = true
            PTKidsBITRobot.motorStop()
        }
    }

    /** จริง ขณะหุ่นกำลังวิ่งหรือทำภารกิจอยู่ */
    //% block="หุ่นกำลังทำภารกิจ"
    //% group="หยุด / ทำต่อ" weight=89
    export function isRunning(): boolean {
        return moving || missionRunning
    }

    /** เปิด/ปิด การกดปุ่ม A เพื่อหยุดหุ่นขณะวิ่ง (เปิดไว้ตั้งแต่เริ่ม) */
    //% block="ปุ่ม A หยุดหุ่นขณะวิ่ง $on"
    //% on.defl=true
    //% group="หยุด / ทำต่อ" weight=88
    export function setStopButtonA(on: boolean): void {
        stopButtonA = on
    }

    //% blockHidden=true
    export function pressStopForTest(): void {
        stopRequested = true
        swallowUndo = true
        stopAt = control.millis()
    }

    /**
     * ลดแขนและอ้ามือตอนเริ่มช่วงสุดท้ายก่อนถึงกล่อง รวมกล่องเหลือง (วิ่งปกติแขนยกไว้) · ใช้กับ "ไปที่" กล่อง
     */
    //% block="ลดแขนก่อนถึงกล่อง $on"
    //% on.defl=true
    //% group="เขียนเส้นทางเอง" weight=60
    export function setLowerArmBeforeBox(on: boolean): void {
        lowerArmBeforeBox = on
    }

    //% blockHidden=true
    export function clearBoxColors(): void {
        for (let i = 0; i < 8; i++) boxColor[i] = -1
    }

    //% blockHidden=true
    export function setDriveHookForTest(f: (p: number) => boolean): void {
        testDrive = f
        useTestDrive = true
    }

    //% blockHidden=true
    export function clearDriveHookForTest(): void {
        useTestDrive = false
    }
}
