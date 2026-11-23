/**
 * ลำดับเก็บกล่องที่เร็วที่สุด (ส่งกล่องขาวเข้าช่องขาว กล่องดำเข้าช่องดำ แล้วกล่องเหลืองกลับ START)
 * Fastest box order for the colour-zone rules, computed on the micro:bit (branch-and-bound search over 8 boxes).
 * Box index i: 0 UL, 1 DL, 2 T1, 3 B1, 4 UR, 5 DR, 6 T2, 7 B2. Colour: 0 black, 1 white.
 */
namespace MissionNav {
    const BOX_NODE = [7, 8, 9, 10, 11, 12, 13, 14]   // UL, DL, T1, B1, UR, DR, T2, B2 node ids
    // cost cache: [from 0 START / 1 white zone / 2 black zone][box] and box → its zone
    let fromCost: number[] = []
    let deliverCost: number[] = []
    let endCost: number[] = []   // zone (1 white / 2 black) → yellow → START
    let cachedColors = ""

    function routeCost(from: number, heading: number, to: number): number {
        const saved = MissionNav.getPresentMask()
        MissionNav.setPresentMask(0)
        MissionNav.plan(from, heading, to)
        const c = MissionNav.lastCost()
        MissionNav.setPresentMask(saved)
        return c
    }

    export function zoneOf(color: number): number {
        return color == 1 ? MissionNav.WZ : MissionNav.BZ
    }

    function fill(colors: number[]): void {
        const key = colors.join(",")
        if (key == cachedColors) return
        fromCost = []
        const starts = [MissionNav.S, MissionNav.WZ, MissionNav.BZ]
        for (let f = 0; f < 3; f++) {
            const h = f == 0 ? 0 : MissionNav.startHeading(starts[f])
            for (let i = 0; i < 8; i++) fromCost.push(routeCost(starts[f], h, BOX_NODE[i]))
        }
        deliverCost = []
        for (let i = 0; i < 8; i++) deliverCost.push(routeCost(BOX_NODE[i], MissionNav.startHeading(BOX_NODE[i]), zoneOf(colors[i])))
        endCost = [0, 0, 0]
        for (let z = 1; z <= 2; z++) {
            const zn = z == 1 ? MissionNav.WZ : MissionNav.BZ
            endCost[z] = routeCost(zn, MissionNav.startHeading(zn), MissionNav.Y) + routeCost(MissionNav.Y, MissionNav.startHeading(MissionNav.Y), MissionNav.S)
        }
        cachedColors = key
    }

    /** where the robot is after delivering box i: 1 white zone, 2 black zone */
    function zoneIndex(colors: number[], i: number): number { return colors[i] == 1 ? 1 : 2 }

    /** estimated seconds for a box order (from START, all listed boxes, then yellow → START) */
    export function orderCost(colors: number[], order: number[]): number {
        fill(colors)
        let at = 0, total = 0
        for (let k = 0; k < order.length; k++) {
            const i = order[k]
            total += fromCost[at * 8 + i] + deliverCost[i]
            at = zoneIndex(colors, i)
        }
        total += at == 0 ? routeCost(MissionNav.S, 0, MissionNav.Y) + routeCost(MissionNav.Y, MissionNav.startHeading(MissionNav.Y), MissionNav.S) : endCost[at]
        return total
    }

    let bestCost = 0
    let bestSeq: number[] = []
    let seq: number[] = []
    let dfsColors: number[] = []
    export let dfsVisits = 0

    let lowBox: number[] = []     // cheapest possible trip for each box (from any start)
    let lowEnd = 0

    function dfs(left: number, at: number, cost: number, depth: number): void {
        dfsVisits++
        // lower bound: every box still left costs at least its cheapest trip, plus the cheapest finish
        let bound = at == 0 ? 0 : cost + lowEnd
        if (at != 0) for (let i = 0; i < 8; i++) if (left & (1 << i)) bound += lowBox[i]
        if (at != 0 && bound >= bestCost - 0.5) return               // cannot beat the best order by at least 0.5 s
        if (left == 0) {
            const total = cost + endCost[at]
            if (total < bestCost) { bestCost = total; bestSeq = seq.slice(0, depth) }
            return
        }
        for (let i = 0; i < 8; i++) {
            if ((left & (1 << i)) == 0) continue
            seq[depth] = i
            dfs(left & ~(1 << i), zoneIndex(dfsColors, i), cost + fromCost[at * 8 + i] + deliverCost[i], depth + 1)
        }
    }

    /** fastest order of the boxes not yet done (bit i of doneMask = box i delivered), starting at START.
     *  Depth-first search with pruning: almost no memory (fits a micro:bit V1). */
    export function bestOrder(colors: number[], doneMask: number): number[] {
        fill(colors)
        dfsColors = colors
        const left = 255 & ~doneMask
        if (left == 0) return []
        lowBox = []
        for (let i = 0; i < 8; i++) {
            // after the first trip every trip starts in a colour zone
            const m = Math.min(fromCost[8 + i], fromCost[16 + i])
            lowBox.push(m + deliverCost[i])
        }
        lowEnd = Math.min(endCost[1], endCost[2])
        bestCost = 1000000
        bestSeq = []
        seq = [0, 0, 0, 0, 0, 0, 0, 0]
        dfs(left, 0, 0, 0)
        return bestSeq
    }
}
