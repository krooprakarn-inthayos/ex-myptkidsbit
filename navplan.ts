/**
 * ตัวหาเส้นทาง: แปลง "ไปจาก A ถึง B" เป็นลำดับคำสั่งวิ่ง
 * Route planner: Dijkstra over (node, heading), then compiles the path into actions.
 *
 * Actions are a flat array of [op, arg] pairs:
 *   1 UTURN  arg = place kind where the U-turn happens
 *   2 EXIT   arg = 0 (drive out of START / a zone onto the line)
 *   3 FOLLOW arg = n (follow the line, stop on the n-th junction)
 *   4 TURN   arg = 0 left, 1 right (at the junction the robot is stopped on)
 *   5 ARRIVE arg = kind + 10 × junctions passed straight first
 */
namespace MissionNav {
    export const OP_UTURN = 1
    export const OP_EXIT = 2
    export const OP_FOLLOW = 3
    export const OP_TURN = 4
    export const OP_ARRIVE = 5

    // cost model (seconds): matches the route simulator's defaults
    const SPEED_CM_S = 25
    const COST_TURN = 0.8
    const COST_CROSS = 0.25
    const COST_UTURN = 2.0
    const BIG = 1000000

    let lastHeading = 0

    /** heading the robot faces after the last planned route */
    export function finalHeading(): number {
        return lastHeading
    }

    /**
     * Plan a route.
     * @param start start node, @param heading heading the robot faces at start, @param goal goal node
     */
    export function plan(start: number, heading: number, goal: number): number[] {
        const acts: number[] = []
        lastHeading = heading
        if (start == goal) return acts

        const S4 = MissionNav.NODE_COUNT * 4
        const dist: number[] = []
        const prev: number[] = []
        const done: boolean[] = []
        for (let i = 0; i < S4; i++) { dist.push(BIG); prev.push(-1); done.push(false) }

        // start states: as placed, or after an initial U-turn
        let forcedUturn = MissionNav.neighbor(start, heading) < 0
        let s0 = start, h0 = heading
        if (forcedUturn) {
            if (start == MissionNav.Y) { s0 = MissionNav.ST }
            h0 = (heading + 2) % 4
            dist[s0 * 4 + h0] = 0
        } else {
            dist[start * 4 + heading] = 0
            if (MissionNav.kind(start) == MissionNav.K_JUNCTION) {
                // optional U-turn on a junction (spin 180° over the cross)
                dist[start * 4 + (heading + 2) % 4] = COST_UTURN
            }
        }

        let best = -1
        while (true) {
            let u = -1, du = BIG
            for (let i = 0; i < S4; i++) if (!done[i] && dist[i] < du) { du = dist[i]; u = i }
            if (u < 0) break
            done[u] = true
            const n = Math.idiv(u, 4), h = u % 4
            if (n == goal) { best = u; break }
            for (let oh = 0; oh < 4; oh++) {
                if (oh == (h + 2) % 4) continue // no reversing mid-route
                const m = MissionNav.neighbor(n, oh)
                if (m < 0) continue
                if (m == MissionNav.Y && goal != MissionNav.Y) continue
                let c = du + MissionNav.edgeLength(n, m) / SPEED_CM_S
                if (oh != h) c += COST_TURN
                else if (MissionNav.isJunction(n) && n != s0) c += COST_CROSS
                const v = m * 4 + oh
                if (c < dist[v]) { dist[v] = c; prev[v] = u }
            }
        }
        if (best < 0) return acts

        // rebuild state path
        const path: number[] = []
        let cur = best
        while (cur >= 0) { path.insertAt(0, cur); cur = prev[cur] }
        const first = path[0]
        const usedUturn = forcedUturn || Math.idiv(first, 4) == start && first % 4 != heading

        if (usedUturn) { acts.push(OP_UTURN); acts.push(MissionNav.kind(start)) }
        else if (MissionNav.kind(start) == MissionNav.K_ZONE) { acts.push(OP_EXIT); acts.push(0) }

        const k = path.length - 1
        const H = (i: number): number => path[i] % 4
        const NODE = (i: number): number => Math.idiv(path[i], 4)
        // turning right where the robot already stands
        if (k >= 1 && H(1) != H(0)) { acts.push(OP_TURN); acts.push(turnDir(H(0), H(1))) }

        let count = 0
        for (let i = 1; i <= k; i++) {
            const n = NODE(i)
            if (i == k) {
                const kd = MissionNav.kind(n)
                if (kd == MissionNav.K_JUNCTION) { count++; acts.push(OP_FOLLOW); acts.push(count) }
                else if (kd == MissionNav.K_YELLOW) {
                    if (count > 0) { acts.push(OP_FOLLOW); acts.push(count) }
                    acts.push(OP_ARRIVE); acts.push(MissionNav.K_YELLOW)
                } else { acts.push(OP_ARRIVE); acts.push(kd + 10 * count) }
                break
            }
            if (MissionNav.isJunction(n)) count++
            if (H(i + 1) != H(i)) {
                acts.push(OP_FOLLOW); acts.push(count)
                acts.push(OP_TURN); acts.push(turnDir(H(i), H(i + 1)))
                count = 0
            }
        }
        lastHeading = H(k)
        return acts
    }

    function turnDir(from: number, to: number): number {
        return ((to - from + 4) % 4) == 1 ? 1 : 0
    }
}
