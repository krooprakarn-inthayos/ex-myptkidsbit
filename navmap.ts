/**
 * แผนที่สนามภารกิจส่งด่วน (มุมมองหุ่นที่ START หันหน้าเข้าสนาม)
 * Express-delivery field map, robot's point of view.
 * Coordinates: pixels measured on the organizer's field image (≈ 8.15 px per cm).
 * Headings: 0 = forward (into field), 1 = robot-right (black-zone side),
 *           2 = back (toward START), 3 = robot-left (white-zone side).
 */

enum MissionPoint {
    //% block="START"
    START = 0,
    //% block="ช่องขาว"
    WhiteZone = 1,
    //% block="ช่องดำ"
    BlackZone = 2,
    //% block="J1"
    J1 = 3,
    //% block="J2"
    J2 = 4,
    //% block="J3"
    J3 = 5,
    //% block="J4"
    J4 = 6,
    //% block="UL"
    UL = 7,
    //% block="DL"
    DL = 8,
    //% block="T1"
    T1 = 9,
    //% block="B1"
    B1 = 10,
    //% block="UR"
    UR = 11,
    //% block="DR"
    DR = 12,
    //% block="T2"
    T2 = 13,
    //% block="B2"
    B2 = 14,
    //% block="เหลือง"
    Yellow = 15
}

namespace MissionNav {
    // ---- node ids ----
    export const S = 0
    export const WZ = 1
    export const BZ = 2
    export const J1 = 3
    export const J2 = 4
    export const J3 = 5
    export const J4 = 6
    export const UL = 7
    export const DL = 8
    export const T1 = 9
    export const B1 = 10
    export const UR = 11
    export const DR = 12
    export const T2 = 13
    export const B2 = 14
    export const Y = 15
    export const ST = 16   // stop bar in front of the yellow box (a cross)
    export const STU = 17
    export const STD = 18
    export const A1 = 19   // V1 x white-zone line
    export const A2 = 20   // V1 x black-zone line
    export const V1T = 21
    export const V1B = 22
    export const WE = 23   // end of white-zone line (gap)
    export const BE = 24
    export const S2 = 25   // V2 x top short line
    export const U2 = 26   // V2 x upper row
    export const D2 = 27
    export const s2 = 28   // V2 x bottom short line
    export const S3 = 29
    export const U3 = 30
    export const D3 = 31
    export const s3 = 32
    export const U4 = 33
    export const D4 = 34
    export const V4T = 35
    export const V4B = 36
    export const SL = 37
    export const SR = 38
    export const SLb = 39
    export const SRb = 40
    export const NODE_COUNT = 41

    // place kinds (shared with the action encoding)
    export const K_JUNCTION = 0
    export const K_BOX = 1
    export const K_ZONE = 2
    export const K_YELLOW = 3
    export const K_STUB = 4

    export const NODE_X = [250, 250, 250, 377, 540, 665, 789, 457, 457, 540, 540, 855, 855, 665, 665, 932,
        865, 865, 865, 377, 377, 377, 377, 438, 438, 540, 540, 540, 540, 665, 665, 665, 665, 789, 789, 789, 789,
        478, 727, 478, 727]
    export const NODE_Y = [532, 152, 905, 532, 532, 532, 532, 345, 718, 93, 972, 345, 718, 93, 972, 532,
        532, 495, 570, 152, 905, 90, 968, 152, 905, 158, 345, 718, 905, 158, 345, 718, 905, 345, 718, 285, 780,
        158, 158, 905, 905]

    // edges as pairs of node ids; the ST–Y edge has no printed line (yellow box sits past the stop bar)
    const EDGES = [
        S, J1, J1, J2, J2, J3, J3, J4, J4, ST, ST, Y, STU, ST, ST, STD,
        V1T, A1, A1, J1, J1, A2, A2, V1B, WZ, A1, A1, WE, BZ, A2, A2, BE,
        T1, S2, S2, U2, U2, J2, J2, D2, D2, s2, s2, B1,
        T2, S3, S3, U3, U3, J3, J3, D3, D3, s3, s3, B2,
        V4T, U4, U4, J4, J4, D4, D4, V4B,
        UL, U2, U2, U3, U3, U4, U4, UR,
        DL, D2, D2, D3, D3, D4, D4, DR,
        SL, S2, S2, S3, S3, SR, SLb, s2, s2, s3, s3, SRb
    ]

    let adj: number[] = []
    let built = false

    function headingOf(dx: number, dy: number): number {
        if (dx > 0) return 0
        if (dy > 0) return 1
        if (dx < 0) return 2
        return 3
    }

    function build(): void {
        adj = []
        for (let i = 0; i < NODE_COUNT * 4; i++) adj.push(-1)
        for (let e = 0; e < EDGES.length; e += 2) {
            const a = EDGES[e], b = EDGES[e + 1]
            const h = headingOf(NODE_X[b] - NODE_X[a], NODE_Y[b] - NODE_Y[a])
            adj[a * 4 + h] = b
            adj[b * 4 + (h + 2) % 4] = a
        }
        built = true
    }

    /** neighbor of `node` in direction `heading`, or -1 */
    export function neighbor(node: number, heading: number): number {
        if (!built) build()
        return adj[node * 4 + heading]
    }

    export function degree(node: number): number {
        let d = 0
        for (let h = 0; h < 4; h++) if (neighbor(node, h) >= 0) d++
        return d
    }

    /** a printed crossing the line sensors detect (≥ 3 printed arms) */
    export function isJunction(node: number): boolean {
        if (node == ST) return true
        return degree(node) >= 3
    }

    export function kind(node: number): number {
        if (node == S || node == WZ || node == BZ) return K_ZONE
        if (node == Y) return K_YELLOW
        if (node >= UL && node <= B2) return K_BOX
        if (isJunction(node)) return K_JUNCTION
        return K_STUB
    }

    /** length of the edge a→b in cm */
    export function edgeLength(a: number, b: number): number {
        const dx = NODE_X[b] - NODE_X[a], dy = NODE_Y[b] - NODE_Y[a]
        return Math.sqrt(dx * dx + dy * dy) / 8.15
    }

    /** menu point → node id (the enum values equal the node ids) */
    export function pointNode(p: number): number {
        return p
    }

    /** heading assumed when the robot is placed at a menu point */
    export function startHeading(p: number): number {
        const n = pointNode(p)
        if (kind(n) == K_JUNCTION || n == S) return 0
        // dead end: facing into the end, i.e. the direction it arrived in
        for (let h = 0; h < 4; h++) {
            if (neighbor(n, h) >= 0) return (h + 2) % 4
        }
        return 0
    }
}
