namespace DebounceTests {
    const LINE = [0, 0, 1000, 1000, 0, 0], OFF = [0, 0, 0, 0, 0, 0], WIDE = [1000, 1000, 1000, 1000, 1000, 1000]
    const FOUR = [0, 1000, 1000, 1000, 1000, 0]   // leaving a crossing at an angle: 4 sensors still on
    function j2ToT1(frames: number[][]): void {
        PTKidsMission.setPosition(MissionPoint.J2)
        Mock.frame = LINE; Mock.log = []; Mock.reads = 0
        Mock.frames = frames
        PTKidsMission.goTo(MissionPoint.T1)
    }
    Harness.test("wobble_on_a_crossing_is_not_a_second_crossing", () => {
        PTKidsMission.setApproach(30); PTKidsMission.setApproachStop(6)
        // turn; U2 seen, wobble back onto U2 (FOUR, WIDE), then line, S2, then line to the box
        j2ToT1([LINE, OFF, LINE,  LINE, LINE, WIDE, FOUR, WIDE, LINE, LINE, LINE,  WIDE,  LINE, LINE, LINE, LINE, LINE, LINE])
        PTKidsMission.setApproachStop(0)
        Harness.ok(Mock.reads >= 13, "kept going past the real S2 (stopped after " + Mock.reads + " reads)")
    })
    Harness.test("leaving_the_last_crossing_is_not_the_box_frame", () => {
        // after S2 the sensors read 4-on while leaving it: that is still the crossing, not the box
        j2ToT1([LINE, OFF, LINE,  LINE, LINE, WIDE, LINE, LINE, WIDE, FOUR, FOUR, LINE, LINE, LINE, FOUR, OFF, OFF, OFF])
        Harness.eq(Mock.reads, 15, "stopped on the real box frame, not while leaving S2")
    })
}
