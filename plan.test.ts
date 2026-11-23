namespace PlanTests {
    const N = MissionNav
    // simulate an action list on the map; returns [node, heading] or throws
    function simulate(start: number, heading: number, acts: number[]): number[] {
        let n = start, h = heading
        const step = (): void => { const m = N.neighbor(n, h); if (m < 0) throw new Error("ran off line at " + n); n = m }
        const passJunctions = (k: number): void => {
            let c = 0
            while (c < k) { step(); if (N.isJunction(n)) c++ }
        }
        for (let i = 0; i < acts.length; i += 2) {
            const op = acts[i], arg = acts[i + 1]
            if (op == 6) { const m = N.neighbor(n, (h + 2) % 4); if (m < 0 || !N.isJunction(m)) throw new Error("BACK must reach a crossing from " + n); n = m }
            else if (op == 1) { if (N.kind(n) == N.K_BOX || n == N.Y) throw new Error("U-turn on a box/yellow"); h = (h + 2) % 4 }
            else if (op == 2) { if (N.kind(n) != N.K_ZONE) throw new Error("EXIT outside zone") }
            else if (op == 3) passJunctions(arg)
            else if (op == 4) h = (h + (arg == 1 ? 1 : 3)) % 4
            else if (op == 5) {
                const k = arg % 10, pass = Math.floor(arg / 10)
                if (k == N.K_YELLOW) { if (n != N.ST) throw new Error("yellow not from ST"); step() }
                else {
                    passJunctions(pass)
                    step(); while (N.kind(n) == N.K_JUNCTION) throw new Error("arrive hit junction at " + n)
                    if (N.kind(n) != k) throw new Error("arrived kind " + N.kind(n) + " want " + k)
                }
            } else throw new Error("bad op " + op)
        }
        return [n, h]
    }
    Harness.test("start_to_UL", () => {
        // turn at J3, turn at the upper row, then pass the crossing next to UL straight into the box
        Harness.eq(N.plan(N.S, 0, N.UL), [2, 0, 3, 3, 4, 0, 3, 1, 4, 0, 5, 11])
    })
    Harness.test("UL_to_start", () => {
        // back onto U2 (facing UL), turn left, stop J2, turn right, pass J1, arrive START
        Harness.eq(N.plan(N.UL, N.startHeading(MissionPoint.UL), N.S), [6, 0, 4, 0, 3, 1, 4, 1, 5, 12])
        Harness.eq(N.finalHeading(), 2)
    })
    Harness.test("T1_to_J2", () => {
        // back onto S2 (facing T1), U-turn on the crossing, stop on the 2nd crossing (U2, J2)
        Harness.eq(N.plan(N.T1, N.startHeading(MissionPoint.T1), N.J2), [6, 0, 1, 0, 3, 2])
    })
    Harness.test("Y_to_start", () => {
        Harness.eq(N.plan(N.Y, N.startHeading(MissionPoint.Yellow), N.S), [6, 0, 1, 0, 5, 42])
    })
    Harness.test("start_to_Y", () => {
        const a = N.plan(N.S, 0, N.Y)
        Harness.eq(a.slice(a.length - 4), [3, 5, 5, 3])
    })
    Harness.test("same_point", () => {
        Harness.eq(N.plan(N.J2, 0, N.J2), [])
        Harness.eq(N.plan(N.T1, N.startHeading(MissionPoint.T1), N.T1), [])
    })
    Harness.test("boxes_are_always_entered_straight", () => {
        // never TURN and then ARRIVE at a box with no crossing passed in between (the body would sweep the box)
        for (let a = 0; a < 16; a++) for (let b = 0; b < 16; b++) {
            if (N.kind(b) != N.K_BOX) continue
            const acts = N.plan(a, N.startHeading(a), b)
            for (let i = 0; i + 3 < acts.length; i += 2) {
                const bad = acts[i] == 4 && acts[i + 2] == 5 && acts[i + 3] == N.K_BOX
                Harness.ok(!bad, "pair " + a + "->" + b + " turns right next to the box: " + JSON.stringify(acts))
            }
            if (acts.length >= 2 && acts[0] != 6) {
                // a plan that starts with a turn and then arrives is also forbidden
                Harness.ok(!(acts[0] == 4 && acts[2] == 5 && acts[3] == N.K_BOX), "pair " + a + "->" + b)
            }
        }
    })
    Harness.test("zone_left_by_backing_to_crossing", () => {
        // white zone → DL: reverse onto A1 (no U-turn in the zone), turn left, J1 left, J3 right, lower row right, pass 1, box
        Harness.eq(N.plan(N.WZ, N.startHeading(MissionPoint.WhiteZone), N.DL),
            [6, 2, 4, 0, 3, 1, 4, 0, 3, 2, 4, 1, 3, 1, 4, 1, 5, 11])
    })
    Harness.test("every_pair", () => {
        for (let a = 0; a < 16; a++) for (let b = 0; b < 16; b++) {
            const h0 = N.startHeading(a)
            const acts = N.plan(a, h0, b)
            const fh = N.finalHeading()
            const r = simulate(a, h0, acts)
            Harness.eq(r[0], b, "pair " + a + "->" + b + " node")
            if (a != b) Harness.eq(r[1], fh, "pair " + a + "->" + b + " heading")
        }
    })
    Harness.test("auto_mission_routes_all_layouts", () => {
        const BOX = [N.UL, N.DL, N.T1, N.B1, N.UR, N.DR, N.T2, N.B2]
        let bad = 0
        for (let m = 0; m < 256; m++) {
            let ones = 0
            for (let i = 0; i < 8; i++) if (m & (1 << i)) ones++
            if (ones != 4) continue
            const colors: number[] = []
            for (let i = 0; i < 8; i++) colors.push((m >> i) & 1)
            const order = N.bestOrder(colors, 0)
            let n = N.S, h = 0, done = 0
            const leg = (goal: number, mask: number): void => {
                N.setPresentMask(mask)
                const acts = N.plan(n, h, goal)
                const fh = N.finalHeading()
                let r: number[] = []
                try { r = simulate(n, h, acts) } catch (e) { bad++; if (bad < 6) console.log("layout " + colors.join("") + " " + n + "->" + goal + " mask " + mask + ": " + e + " acts " + acts.join(",")); N.setPresentMask(0); return }
                if (r[0] != goal || r[1] != fh) { bad++; if (bad < 6) console.log("layout " + colors.join("") + " " + n + "->" + goal + " ends " + r.join(",") + " want " + goal + "," + fh + " acts " + acts.join(",")) }
                n = goal; h = fh
            }
            for (const i of order) {
                leg(BOX[i], 255 & ~done)
                leg(colors[i] == 1 ? N.WZ : N.BZ, 255 & ~done & ~(1 << i))
                done |= 1 << i
            }
            leg(N.Y, 0); leg(N.S, 0)
        }
        N.setPresentMask(0)
        Harness.eq(bad, 0, "bad legs")
    })
}
