namespace GripperTests {
    const LINE = [0, 0, 1000, 1000, 0, 0], OFF = [0, 0, 0, 0, 0, 0], WIDE = [1000, 1000, 1000, 1000, 1000, 1000]
    Harness.test("straight_entry_needs_no_gripper_moves", () => {
        // with straight box entry the old "lift on the turn into a box" never triggers
        PTKidsMission.setGripper(Servo_Write.P8, 90, 0, Servo_Write.P12, 180, 0)
        PTKidsMission.setPosition(MissionPoint.START)
        Mock.frame = LINE; Mock.log = []
        Mock.frames = [WIDE, LINE,                 // exit START
                       LINE, WIDE, LINE, LINE, WIDE, LINE, LINE, WIDE,   // J1, J2, J3
                       LINE, OFF, LINE,            // turn left at J3
                       LINE, WIDE,                 // stop on the upper row
                       LINE, OFF, LINE,            // turn left
                       LINE, WIDE, LINE, LINE, LINE, WIDE]   // pass the crossing next to UL, arrive at UL frame
        PTKidsMission.goTo(MissionPoint.UL)
        PTKidsMission.clearGripper()
        Harness.eq(PTKidsMission.currentPosition(), MissionPoint.UL)
        Harness.ok(Mock.log.filter(e => e.indexOf("servo") == 0).length == 0, "no servo commands")
    })
}
