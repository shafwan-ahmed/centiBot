"""
Click-to-navigate for TWO N20 DC-motor bots, driven together from one camera.

Controls (video window):
    Left click   = add a target; clicks alternate between the bots:
                       click 1 -> Bot 1, click 2 -> Bot 2,
                       click 3 -> Bot 1, click 4 -> Bot 2, ...
    Right click  = undo the last click (only if that target isn't in use yet)
    c            = clear all targets (before starting)
    s            = start the mission
    q            = quit / abort (stops both bots)

HOW THE TARGETS ARE PAIRED
    Each bot has its own target queue. "Pair k" = the k-th target of Bot 1
    together with the k-th target of Bot 2. Because clicks alternate, pair k
    is complete as soon as you have clicked 2k+2 times.

HOW THE COORDINATION WORKS (Mission class)
    AWAITING_TARGETS -> MOVING -> WAITING -> (next pair) AWAITING_TARGETS ...

    AWAITING_TARGETS : the next pair isn't fully clicked yet; nothing moves.
    MOVING           : each bot runs its own navigation state machine toward
                       its own target. A bot that arrives early simply stays
                       still while the other one keeps moving.
    WAITING          : entered only when BOTH bots have arrived. Lasts
                       WAIT_TIME seconds, then the mission moves to the next
                       pair automatically (no extra clicks needed if it was
                       already clicked; otherwise it waits for the clicks).

    Nothing blocks: the loop keeps reading the camera and talking to both
    bots every frame, including during the wait.

COLLISION AVOIDANCE (Mission._update_avoidance)
    While MOVING, every pair of bots is watched. If two bots get closer than
    SAFE_DISTANCE_PX and one is heading toward the other:
      * the bot that is still moving and has the lower priority (the bot whose
        ID is YIELDING_BOT_ID) STOPS and holds position ("YIELDING");
      * the other bot gets a temporary side-step waypoint (the yellow ring)
        that takes it around the stopped bot, then carries on to its target;
      * if the other bot has already arrived and is parked, it is simply an
        obstacle: the moving bot goes around it and the parked one never moves.
    The stopped bot resumes once the bots are farther apart than
    CLEAR_DISTANCE_PX (or the other bot has parked, or MAX_YIELD_S passed).

PER-BOT NAVIGATION (Robot class) is the original tuned single-bot logic:
    measure (bot stopped) -> ONE ESP-timed pulse -> brake -> settle -> measure,
    with smooth overlapping pulses ("DRIVE") for long straight legs.
    Each bot keeps its own learned turn/move rates and stiction power.

Packet format: <letter><percent>T<ms>, e.g. "D50T120" = rotate right 50% for
120 ms.  W forward, S backward, A rotate left, D rotate right, X stop.

Each bot is identified in the camera by the ArUco marker whose ID equals the
bot's ID (Bot 1 -> marker 1, Bot 2 -> marker 2).

Setup:
    pip install opencv-contrib-python numpy
"""

import math
import socket
import time
from dataclasses import dataclass

import cv2

# =====================================================================
# CONFIGURATION  (everything you're likely to change lives in this block)
# =====================================================================

# ---------- Robots ----------
@dataclass
class BotConfig:
    bot_id: int          # also the ArUco marker ID on top of the bot
    ip: str
    port: int = 4210
    color: tuple = (255, 0, 0)   # BGR, used for drawing


ROBOTS = [
    BotConfig(bot_id=1, ip="192.168.50.100", color=(255, 0, 0)),   # blue
    BotConfig(bot_id=2, ip="192.168.50.175", color=(0, 200, 0)),   # green
]

# ---------- Coordination ----------
WAIT_TIME = 3          # seconds both bots stay still after BOTH have arrived
AUTO_START = False     # True = begin as soon as the first pair is clicked (no 's')

# ---------- Collision avoidance ----------
AVOID_COLLISIONS = True
YIELDING_BOT_ID = 2          # when both bots are moving, THIS bot stops and the other goes around
SAFE_DISTANCE_PX = 100       # closer than this (and heading at each other) = conflict
                             #   -> roughly 2-3x the bot's length as seen in the camera
CLEAR_DISTANCE_PX = 130      # farther than this = conflict over (keep > SAFE_DISTANCE_PX)
FRONT_ANGLE_DEG = 70         # the other bot counts as "in the way" within this cone ahead
DETOUR_OFFSET_PX = 120       # how far to the side of the stopped bot the side-step point is
DETOUR_TOLERANCE_PX = 20     # side-step point counts as reached within this distance
DETOUR_EDGE_MARGIN_PX = 30   # keep side-step points this far inside the image
MAX_YIELD_S = 15             # safety: a stopped bot never waits longer than this

# ---------- Camera ----------
CAMERA_INDEX = 0       # change if the wrong camera opens
FRAME_WIDTH = 700
FRAME_HEIGHT = 505
WINDOW_NAME = "Navigation"

# ---------- Tolerances ----------
POSITION_TOLERANCE_PX = 5       # distance at which a bot counts as "arrived"
ANGLE_TOLERANCE_DEG = 8
ROTATE_SIGN = 1                 # confirmed correct via testing

# ---------- Pulse control ----------
BRAKE_MS = 80                   # keep equal to BRAKE_MS in the firmware
SETTLE_MS = 200                 # extra wait for camera/bot to settle before measuring
AVG_FRAMES = 3                  # frames averaged per measurement (noise filter)
PULSE_AIM = 0.7                 # aim for 70% of the estimated pulse, finish in the next one
MIN_PULSE_MS = 40
MAX_TURN_PULSE_MS = 250
MAX_MOVE_PULSE_MS = 400

START_TURN_PERCENT = 50
START_MOVE_PERCENT = 50
PERCENT_BUMP = 15               # added when a pulse produced no motion (stiction)
MAX_PULSE_PERCENT = 90

TURN_RATE_INIT = 0.10           # deg per ms - learned after the first pulses
MOVE_RATE_INIT = 0.10           # px per ms  - learned after the first pulses
RATE_MIN, RATE_MAX = 0.02, 0.5
MIN_TURN_MOTION_DEG = 1.0       # less than this = "didn't move"
MIN_MOVE_MOTION_PX = 3.0

REVERSE_MAX_PX = 60             # target this close and behind us -> back up instead of spinning
REVERSE_ANGLE_DEG = 150

# ---------- Smooth driving (rolling pulses) ----------
DRIVE_MIN_PX = 50               # farther than this -> smooth drive; closer -> fine pulses
DRIVE_PULSE_MS = 120            # each rolling pulse (the bot's fail-safe window)
DRIVE_RESEND_MS = 50            # re-send interval; < DRIVE_PULSE_MS so pulses overlap
DRIVE_LEAD_MS = 150             # stop this many ms of travel early (camera lag + brake)
DRIVE_CONFIRM_FRAMES = 2        # consecutive bad-heading frames before aborting a drive
DRIVE_LOST_GRACE_MS = 200       # marker may vanish this long before the drive is cut
DRIVE_STICTION_CHECK_MS = 600   # no motion after this long -> stop, let learn() bump power
DRIVE_MAX_MS = 6000             # safety cap on one continuous drive
DRIVE_RATE_INIT = 0.10          # px per ms during a long drive (learned)

# =====================================================================
# HELPERS
# =====================================================================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def calculate_angle(corners):
    top_left, top_right, _, _ = corners
    vector = top_right - top_left
    return math.degrees(math.atan2(vector[1], vector[0]))


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def average_pose(samples):
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    sin_sum = sum(math.sin(math.radians(s[2])) for s in samples)
    cos_sum = sum(math.cos(math.radians(s[2])) for s in samples)
    return (sum(xs) / len(xs), sum(ys) / len(ys), math.degrees(math.atan2(sin_sum, cos_sum)))


def detect_poses(img):
    """Return {marker_id: (x, y, heading_deg)} for every marker in the frame."""
    corners, ids, _ = detector.detectMarkers(img)
    poses = {}
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(img, corners)
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            pts = marker_corners[0]
            cx, cy = pts.mean(axis=0)
            poses[int(marker_id)] = (float(cx), float(cy), calculate_angle(pts))
    return poses


# =====================================================================
# ONE ROBOT: communication + navigation to a single target
# =====================================================================

class Robot:
    """Everything that belongs to ONE bot: its address, target queue, learned
    rates and navigation state machine. Two instances run side by side."""

    def __init__(self, cfg: BotConfig):
        self.cfg = cfg
        self.id = cfg.bot_id
        self.name = f"Bot {cfg.bot_id}"
        self.color = cfg.color

        self.targets = []              # this bot's queue of (x, y) targets
        self.target = None             # target it is currently heading to
        self.arrived = False
        self.pose = None               # latest (x, y, heading) or None if not visible
        self.detour = None             # temporary side-step waypoint (collision avoidance)
        self.paused = False            # True while yielding to another bot

        self.phase = "SETTLE"          # SETTLE (wait) -> SAMPLE (average frames) -> decide -> SETTLE
        self.ready_at = 0.0
        self.samples = []
        self.pending = None            # last pulse, so we can learn from what it did
        self.last_kind = "turn"

        self.turn_rate = TURN_RATE_INIT
        self.move_rate = MOVE_RATE_INIT
        self.drive_rate = DRIVE_RATE_INIT
        self.turn_percent = START_TURN_PERCENT
        self.move_percent = START_MOVE_PERCENT

        self.drive_start_t = 0.0       # smooth-drive bookkeeping
        self.drive_start_pose = None
        self.last_send = 0.0
        self.last_seen = 0.0
        self.bad_frames = 0

        self.debug = ""
        self.status = ""

    # ----- communication (each bot only ever talks to its own IP) -----
    def send_raw(self, msg: str):
        sock.sendto(msg.encode(), (self.cfg.ip, self.cfg.port))

    def send_stop(self):
        self.send_raw('X')

    def send_pulse(self, cmd: str, percent: int, ms: int):
        self.send_raw(f"{cmd}{int(percent)}T{int(ms)}")

    # ----- target handling -----
    @property
    def goal(self):
        """What the bot is steering toward right now: the detour point if there
        is one, otherwise its real target."""
        return self.detour or self.target

    def assign_target(self, target, now):
        """Begin navigating to a new target. Learned rates/power are kept."""
        self.target = target
        self.arrived = False
        self.detour = None
        self.paused = False
        self.phase = "SETTLE"
        self.ready_at = now + 0.3
        self.samples = []
        self.pending = None
        self.last_kind = "turn"
        self.status = ""

    # ----- collision-avoidance hooks (called by Mission) -----
    def pause(self, now):
        """Stop right now and hold still (yielding to another bot)."""
        self.send_stop()
        self.paused = True
        self.pending = None            # the interrupted move must not be 'learned' from
        self.phase = "SETTLE"
        self.status = "YIELDING"

    def resume(self, now):
        """Carry on after a pause: settle, then re-measure and decide afresh."""
        self.paused = False
        self.samples = []
        self.last_kind = "turn"
        self.ready_at = now + (BRAKE_MS + SETTLE_MS) / 1000.0
        self.phase = "SETTLE"
        self.status = ""

    def set_detour(self, point, now):
        """Steer to a side-step point before continuing to the real target."""
        self.detour = point
        self.status = "DETOUR"
        if self.phase == "DRIVE":
            self.end_drive(now)        # stop the current run; re-plan toward the detour
        else:
            self.samples = []

    # ----- per-frame step -----
    def step(self, pose, now):
        self.pose = pose
        if self.target is None or self.arrived or self.paused:
            return  # idle: parked at its target, yielding, or nothing assigned yet
        if self.phase == "DRIVE":
            self.drive_step(pose, now)
            return
        if self.phase == "SETTLE":
            if now >= self.ready_at:
                self.phase = "SAMPLE"
                self.samples = []
            return
        if pose is None:
            return  # marker lost: nothing in flight (pulses brake themselves), just wait
        self.samples.append(pose)
        if len(self.samples) < AVG_FRAMES:
            return
        self.decide(average_pose(self.samples), now)

    def decide(self, pose, now):
        self.learn(pose)
        cx, cy, ang = pose

        # reached the side-step point? then go back to the real target
        if self.detour and math.hypot(self.detour[0] - cx, self.detour[1] - cy) <= DETOUR_TOLERANCE_PX:
            self.detour = None

        tx, ty = self.goal
        distance = math.hypot(tx - cx, ty - cy)
        desired = math.degrees(math.atan2(ty - cy, tx - cx))
        herr = ROTATE_SIGN * normalize_angle(desired - ang)
        self.debug = f"dist:{distance:.0f} herr:{herr:.0f} heading:{ang:.0f}"

        # 1) at the target?
        if distance <= POSITION_TOLERANCE_PX:
            self.arrived = True
            self.status = "ARRIVED"
            return

        # 2) target just behind us: back up instead of spinning around
        if distance < REVERSE_MAX_PX and abs(herr) > REVERSE_ANGLE_DEG:
            self.pulse_move('S', distance, pose, now)
            return

        # 3) not facing it: turn (looser tolerance while already driving)
        turn_tol = ANGLE_TOLERANCE_DEG if self.last_kind == "turn" else ANGLE_TOLERANCE_DEG * 2
        if abs(herr) > turn_tol:
            self.pulse_turn(herr, pose, now)
            return

        # 4) facing it: long leg -> smooth drive, short leg -> fine pulse
        if distance > DRIVE_MIN_PX:
            self.start_drive(pose, now)
        else:
            self.pulse_move('W', distance, pose, now)

    # ----- smooth drive -----
    def send_drive_pulse(self, now):
        self.send_pulse('W', self.move_percent, DRIVE_PULSE_MS)
        self.last_send = now

    def start_drive(self, pose, now):
        self.drive_start_t = now
        self.drive_start_pose = pose
        self.last_seen = now
        self.bad_frames = 0
        self.last_kind = "move"
        self.phase = "DRIVE"
        self.send_drive_pulse(now)

    def drive_step(self, pose, now):
        if pose is None:
            if now - self.last_seen > DRIVE_LOST_GRACE_MS / 1000.0:
                self.end_drive(now)
            return
        self.last_seen = now
        cx, cy, ang = pose

        tx, ty = self.goal
        distance = math.hypot(tx - cx, ty - cy)
        herr = ROTATE_SIGN * normalize_angle(math.degrees(math.atan2(ty - cy, tx - cx)) - ang)
        self.debug = f"dist:{distance:.0f} herr:{herr:.0f} heading:{ang:.0f}"
        self.status = f"{'DETOUR ' if self.detour else ''}DRIVE W {self.move_percent}%"

        # 0) reached the side-step point: stop, then re-plan toward the real target
        if self.detour and distance <= DETOUR_TOLERANCE_PX:
            self.detour = None
            self.end_drive(now)
            return

        # 1) near the real target: cut early by the lead distance
        if self.detour is None and distance <= POSITION_TOLERANCE_PX + self.drive_rate * DRIVE_LEAD_MS:
            self.end_drive(now)
            return

        # 2) drifted off course
        if abs(herr) > ANGLE_TOLERANCE_DEG * 2:
            self.bad_frames += 1
            if self.bad_frames >= DRIVE_CONFIRM_FRAMES:
                self.end_drive(now)
                return
        else:
            self.bad_frames = 0

        # 3) not moving (stiction) or running too long
        elapsed = now - self.drive_start_t
        x0, y0, _ = self.drive_start_pose
        if elapsed > DRIVE_STICTION_CHECK_MS / 1000.0 and math.hypot(cx - x0, cy - y0) < MIN_MOVE_MOTION_PX:
            self.end_drive(now)
            return
        if elapsed > DRIVE_MAX_MS / 1000.0:
            self.end_drive(now)
            return

        # 4) keep the rolling pulses overlapping
        if now - self.last_send >= DRIVE_RESEND_MS / 1000.0:
            self.send_drive_pulse(now)

    def end_drive(self, now):
        self.send_stop()  # active brake right now
        ms = (now - self.drive_start_t) * 1000.0
        self.pending = {"kind": "drive", "cmd": 'W', "ms": ms, "pose": self.drive_start_pose}
        self.last_kind = "move"
        self.ready_at = now + (BRAKE_MS + SETTLE_MS) / 1000.0
        self.phase = "SETTLE"

    # ----- pulses -----
    @staticmethod
    def pulse_ms(raw_ms, cap):
        return clamp(PULSE_AIM * raw_ms, MIN_PULSE_MS, cap)

    def pulse_turn(self, err, pose, now):
        cmd = 'D' if err > 0 else 'A'
        ms = self.pulse_ms(abs(err) / self.turn_rate, MAX_TURN_PULSE_MS)
        self.fire(cmd, self.turn_percent, ms, "turn", pose, now)

    def pulse_move(self, cmd, distance, pose, now):
        ms = self.pulse_ms(distance / self.move_rate, MAX_MOVE_PULSE_MS)
        self.fire(cmd, self.move_percent, ms, "move", pose, now)

    def fire(self, cmd, percent, ms, kind, pose, now):
        self.send_pulse(cmd, percent, ms)
        self.pending = {"kind": kind, "cmd": cmd, "ms": ms, "pose": pose}
        self.last_kind = kind
        self.status = f"{cmd} {percent}% {ms:.0f}ms"
        self.ready_at = now + (ms + BRAKE_MS + SETTLE_MS) / 1000.0
        self.phase = "SETTLE"

    def learn(self, pose):
        """Compare where the last pulse left the bot with where it started:
        update the rate estimates, or push harder if it didn't move at all."""
        p = self.pending
        self.pending = None
        if not p:
            return
        x0, y0, a0 = p["pose"]
        x1, y1, a1 = pose

        if p["kind"] == "turn":
            d = ROTATE_SIGN * normalize_angle(a1 - a0)
            moved = d if p["cmd"] == 'D' else -d
            if moved < MIN_TURN_MOTION_DEG:
                self.turn_percent = min(MAX_PULSE_PERCENT, self.turn_percent + PERCENT_BUMP)
            elif p["ms"] >= 50:
                sample = moved / p["ms"]
                self.turn_rate = clamp(0.5 * self.turn_rate + 0.5 * sample, RATE_MIN, RATE_MAX)
        else:
            moved = math.hypot(x1 - x0, y1 - y0)
            if moved < MIN_MOVE_MOTION_PX:
                self.move_percent = min(MAX_PULSE_PERCENT, self.move_percent + PERCENT_BUMP)
            elif p["kind"] == "drive":
                if p["ms"] >= 200:
                    sample = moved / p["ms"]
                    self.drive_rate = clamp(0.5 * self.drive_rate + 0.5 * sample, RATE_MIN, RATE_MAX)
            elif p["ms"] >= 50:
                sample = moved / p["ms"]
                self.move_rate = clamp(0.5 * self.move_rate + 0.5 * sample, RATE_MIN, RATE_MAX)


# =====================================================================
# THE MISSION: click handling + coordination of all bots
# =====================================================================

@dataclass
class Conflict:
    avoider: Robot            # the bot that keeps moving (goes around)
    other: Robot              # the obstacle: the yielding bot, or a parked bot
    yielder: Robot = None     # set only if `other` was moving and had to be stopped
    since: float = 0.0


class Mission:
    IDLE = "IDLE"                          # setup: clicking targets, not started
    AWAITING_TARGETS = "AWAITING TARGETS"  # started, but next pair isn't fully clicked
    MOVING = "MOVING"                      # bots driving to the current pair
    WAITING = "WAITING"                    # both arrived, holding for WAIT_TIME

    def __init__(self, robots):
        self.robots = robots
        self.clicks = []           # click order -> robot that received it (for undo)
        self.pair_index = 0        # which pair of targets is current
        self.state = Mission.IDLE
        self.wait_until = 0.0
        self.conflicts = []        # active collision-avoidance situations
        self.frame_w = FRAME_WIDTH     # updated from the real frame in main()
        self.frame_h = FRAME_HEIGHT

    @property
    def started(self):
        return self.state != Mission.IDLE

    # ----- target assignment (alternating clicks) -----
    def add_click(self, x, y):
        """Click n goes to robot (n mod number_of_robots): Bot 1, Bot 2, Bot 1, ..."""
        robot = self.robots[len(self.clicks) % len(self.robots)]
        robot.targets.append((x, y))
        self.clicks.append(robot)

    def undo_click(self):
        """Remove the last click, unless that target is already being used."""
        if not self.clicks:
            return
        robot = self.clicks[-1]
        idx = len(robot.targets) - 1
        in_use = self.started and (idx < self.pair_index or
                                   (idx == self.pair_index and self.state in (Mission.MOVING, Mission.WAITING)))
        if in_use:
            return
        robot.targets.pop()
        self.clicks.pop()

    def clear(self):
        if self.started:
            return
        for r in self.robots:
            r.targets.clear()
        self.clicks.clear()
        self.pair_index = 0

    # ----- run control -----
    def start(self, now):
        if self.started:
            return
        self.pair_index = 0
        self.state = Mission.AWAITING_TARGETS
        self._try_begin_pair(now)

    def stop(self):
        self.state = Mission.IDLE
        self._reset_avoidance()
        for r in self.robots:
            r.target = None
            r.arrived = False
            r.send_stop()

    def _pair_ready(self):
        """A pair is ready only when EVERY bot has a target at this index."""
        return all(len(r.targets) > self.pair_index for r in self.robots)

    def _try_begin_pair(self, now):
        if self._pair_ready():
            for r in self.robots:
                r.assign_target(r.targets[self.pair_index], now)
            self.state = Mission.MOVING
            print(f"Pair {self.pair_index + 1}: " +
                  ", ".join(f"{r.name} -> {r.target}" for r in self.robots))

    # ----- per-frame update -----
    def update(self, poses, now):
        # every bot keeps being serviced every frame (even while idle/waiting)
        for r in self.robots:
            r.pose = poses.get(r.id)

        if self.state == Mission.AWAITING_TARGETS:
            self._try_begin_pair(now)

        elif self.state == Mission.MOVING:
            if AVOID_COLLISIONS:
                self._update_avoidance(now)
            for r in self.robots:
                r.step(r.pose, now)
            # the wait starts only once EVERY bot has arrived
            if all(r.arrived for r in self.robots):
                self._reset_avoidance()
                for r in self.robots:
                    r.send_stop()          # belt and braces: hold still
                self.wait_until = now + WAIT_TIME
                self.state = Mission.WAITING
                print(f"Both bots arrived. Waiting {WAIT_TIME}s.")

        elif self.state == Mission.WAITING:
            if now >= self.wait_until:
                self.pair_index += 1
                self.state = Mission.AWAITING_TARGETS
                self._try_begin_pair(now)
                if self.state == Mission.AWAITING_TARGETS:
                    print("Waiting for the next pair of clicks.")

    # ----- collision avoidance -----
    @staticmethod
    def _dist(a, b):
        return math.hypot(a.pose[0] - b.pose[0], a.pose[1] - b.pose[1])

    @staticmethod
    def _heading_into(a, b):
        """True if bot `a` is moving and bot `b` is in its way (ahead, toward its goal)."""
        if a.target is None or a.arrived or a.paused or a.pose is None or b.pose is None:
            return False
        px, py = a.pose[0], a.pose[1]
        gx, gy = a.goal
        goal_dist = math.hypot(gx - px, gy - py)
        obst_dist = math.hypot(b.pose[0] - px, b.pose[1] - py)
        if obst_dist > goal_dist + SAFE_DISTANCE_PX:
            return False               # a stops well before reaching b
        to_goal = math.degrees(math.atan2(gy - py, gx - px))
        to_obst = math.degrees(math.atan2(b.pose[1] - py, b.pose[0] - px))
        return abs(normalize_angle(to_goal - to_obst)) < FRONT_ANGLE_DEG

    def _in_conflict(self, a, b):
        return any({c.avoider, c.other} == {a, b} for c in self.conflicts)

    def _update_avoidance(self, now):
        # 1) finish conflicts that have passed -> everyone moves normally again
        for c in list(self.conflicts):
            a, b = c.avoider, c.other
            far = a.pose and b.pose and self._dist(a, b) > CLEAR_DISTANCE_PX
            timed_out = c.yielder is not None and now - c.since > MAX_YIELD_S
            if far or a.arrived or timed_out:
                self.conflicts.remove(c)
                a.detour = None
                if c.yielder is not None:
                    c.yielder.resume(now)
                print(f"Conflict {a.name}/{b.name} passed.")

        # 2) look for new conflicts
        for i, a in enumerate(self.robots):
            for b in self.robots[i + 1:]:
                if self._in_conflict(a, b) or a.pose is None or b.pose is None:
                    continue
                if a.arrived and b.arrived:
                    continue
                d = self._dist(a, b)
                if d >= SAFE_DISTANCE_PX:
                    continue
                if self._heading_into(a, b) or self._heading_into(b, a) or d < SAFE_DISTANCE_PX / 2:
                    self._begin_conflict(a, b, now)

    def _begin_conflict(self, a, b, now):
        # who goes around whom?
        if a.arrived or b.arrived:                   # one is parked: the moving one goes around
            avoider, other = (b, a) if a.arrived else (a, b)
            conflict = Conflict(avoider, other, None, now)
        else:                                        # both moving: YIELDING_BOT_ID stops
            yielder = b if b.id == YIELDING_BOT_ID else a
            avoider = a if yielder is b else b
            yielder.pause(now)
            conflict = Conflict(avoider, yielder, yielder, now)
        self.conflicts.append(conflict)

        # the moving bot sidesteps only if the other bot is actually in its way
        if self._heading_into(avoider, conflict.other):
            point = self._detour_point(avoider, conflict.other)
            if point:
                avoider.set_detour(point, now)
        print(f"Conflict: {avoider.name} goes around {conflict.other.name}"
              + (f" ({conflict.yielder.name} stopped)." if conflict.yielder else " (parked)."))

    def _detour_point(self, avoider, obstacle):
        """A point beside the obstacle, on the side away from the avoider's path."""
        px, py = avoider.pose[0], avoider.pose[1]
        ox, oy = obstacle.pose[0], obstacle.pose[1]
        gx, gy = avoider.target
        ux, uy = gx - px, gy - py
        norm = math.hypot(ux, uy)
        if norm < 1:
            return None
        ux, uy = ux / norm, uy / norm
        nx, ny = -uy, ux                              # unit vector perpendicular to the path
        side = -1 if (ox - px) * nx + (oy - py) * ny > 0 else 1   # opposite the obstacle's side
        m = DETOUR_EDGE_MARGIN_PX
        candidates = [(ox + s * nx * DETOUR_OFFSET_PX, oy + s * ny * DETOUR_OFFSET_PX)
                      for s in (side, -side)]
        for x, y in candidates:                       # prefer a point inside the image
            if m <= x <= self.frame_w - m and m <= y <= self.frame_h - m:
                return (x, y)
        x, y = candidates[0]
        return (clamp(x, m, self.frame_w - m), clamp(y, m, self.frame_h - m))

    def _reset_avoidance(self):
        self.conflicts.clear()
        for r in self.robots:
            r.detour = None
            r.paused = False

    # ----- UI -----
    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.add_click(x, y)
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.undo_click()

    def draw_overlay(self, img, now):
        for r in self.robots:
            for i, (tx, ty) in enumerate(r.targets):
                current = self.started and i == self.pair_index and self.state in (Mission.MOVING, Mission.WAITING)
                cv2.circle(img, (int(tx), int(ty)), 6, r.color, -1)
                if current:
                    cv2.circle(img, (int(tx), int(ty)), 11, (0, 165, 255), 2)
                cv2.putText(img, f"B{r.id}-{i + 1}", (int(tx) + 8, int(ty) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, r.color, 2)
            if self.state == Mission.MOVING and r.target and r.pose and not r.arrived:
                gx, gy = r.goal
                cv2.line(img, (int(r.pose[0]), int(r.pose[1])), (int(gx), int(gy)), r.color, 1)
            if r.detour:
                cv2.circle(img, (int(r.detour[0]), int(r.detour[1])), 8, (0, 255, 255), 2)

        if self.state == Mission.IDLE:
            nxt = self.robots[len(self.clicks) % len(self.robots)].name
            msg = (f"SETUP | next click -> {nxt} | L-click: add  R-click: undo  "
                   f"c: clear  s: start  q: quit")
        else:
            total = min(len(r.targets) for r in self.robots)
            msg = f"RUNNING | pair {self.pair_index + 1}/{max(total, self.pair_index + 1)} | {self.state}"
            if self.state == Mission.WAITING:
                msg += f" {max(0.0, self.wait_until - now):.1f}s"
            msg += "  q: abort"
        cv2.putText(img, msg, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        for i, r in enumerate(self.robots):
            if r.pose is None:
                line = f"{r.name}: marker not visible"
            elif self.state == Mission.IDLE or not (r.debug or r.status):
                line = f"{r.name}: heading {r.pose[2]:.0f}"
            else:
                line = f"{r.name}: {r.debug} | {r.status}"
            cv2.putText(img, line, (10, 50 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, r.color, 2)


# =====================================================================
# MAIN LOOP
# =====================================================================

def main():
    cam = cv2.VideoCapture(CAMERA_INDEX)
    cam.set(3, FRAME_WIDTH)
    cam.set(4, FRAME_HEIGHT)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # keep frames fresh; stale frames = wrong measurements
    if not cam.isOpened():
        raise RuntimeError("Could not open camera. Try a different CAMERA_INDEX (0, 1, 2...).")

    mission = Mission([Robot(cfg) for cfg in ROBOTS])
    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, mission.on_mouse)

    print("Click the video window to set targets (alternating Bot 1, Bot 2, Bot 1, ...).")
    print("Left click: add | Right click: undo | c: clear | s: start | q: quit")

    try:
        while True:
            success, img = cam.read()
            if not success:
                print("Failed to grab frame")
                break

            now = time.time()
            mission.frame_h, mission.frame_w = img.shape[:2]
            poses = detect_poses(img)

            if AUTO_START and not mission.started and mission._pair_ready():
                mission.start(now)
            mission.update(poses, now)

            mission.draw_overlay(img, now)
            cv2.imshow(WINDOW_NAME, img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Stopped.")
                break
            elif key == ord('c'):
                mission.clear()
            elif key == ord('s') and not mission.started:
                mission.start(now)
                print("Mission started.")
    finally:
        mission.stop()  # sends X to every bot
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()