#!/usr/bin/env python3
"""
Replay a recorded RAMMP ROS 2 bag into RammpUI, through the interfaces the robot uses.

Usage:
    ramms-bag info BAG                         # topics, cameras, objects, and what will and will not work
    ramms-bag play BAG                         # real time, into a running RammpUI (PIE, -game or packaged)
    ramms-bag play BAG --rate 2 --start 30 --end 60 --loop
    ramms-bag play BAG --overlay segmentation  # the overlay slot shows the segmenter's masks (default: wrist)
    ramms-bag play BAG --positions derived     # no 3D from the tracker: RammsUI derives markers from depth
    ramms-bag play BAG --verify                # afterwards, check where the 3D markers landed (editor PIE)
    ramms-bag play BAG --dry-run               # no network: what would be sent

BAG is a rosbag2 directory (sqlite3 .db3 files). A zipped bag has to be extracted first.

What it sends:
- RMSS: the scene camera's colour and depth, each with its intrinsics and the camera's pose from the
  bag's TF (relative to the UE stream receiver, which stands for base_link).
- One overlay camera: the wrist camera, the segmenter's masks over the scene frame, or the segmenter's
  own rendering (--overlay).
- Remote Control: every tracked object, as 2D boxes and 3D markers with their staleness, and the robot
  display's joints.

On exit (the end, Ctrl-C or an error) the UI is put back: detections cleared, the overlay window closed,
the joints restored.
"""

from __future__ import annotations

import argparse
import sys


def _ids_from(client, log=print):
    """The UI's own stream ids (RammsCameraStreamSettings), so a reconfigured UI still shows the replay."""
    from ramms_tools.bagplay.player import StreamIds
    from ramms_tools.scripts.ui_bridge_test import BridgeError
    ids = StreamIds()

    def setting(name, default):
        try:
            return client.get_setting(name) or default
        except BridgeError:
            return default
    ids.scene_color = setting("SceneColorStreamID", ids.scene_color)
    ids.overlay_color = setting("OverlayColorStreamID", ids.overlay_color)
    for name, kind, attr in (("SceneDataStreams", "Depth", "scene_depth"),
                             ("OverlayDataStreams", "Depth", "overlay_depth"),
                             ("OverlayDataStreams", "Segmentation", "overlay_segmentation")):
        found = next((d.get("StreamID") for d in setting(name, []) or [] if d.get("Kind") == kind), None)
        if found:
            setattr(ids, attr, found)
    return ids


def cmd_info(args):
    from ramms_tools.bagplay.player import (BagSession, JOINTS, RAW, SCENE_COLOR, SCENE_INFO, TRACKED, WRIST_COLOR,
                                            WRIST_INFO, BASE_FRAME, intrinsics_of)
    from ramms_tools.ros_ue import camera_extrinsic
    s = BagSession(args.bag)
    print(f"{args.bag}: {s.reader.duration_s:.1f} s, {len(s.reader.paths)} file(s)")
    for name, t in sorted(s.topics.items()):
        rate = t.count / max((t.last_ns - t.first_ns) / 1e9, 1e-9) if t.count > 1 else 0.0
        print(f"  {t.count:>6}  {rate:6.1f} Hz  {name}  ({t.type})")
    for label, image_topic, info_topic in (("scene", SCENE_COLOR, SCENE_INFO), ("wrist", WRIST_COLOR, WRIST_INFO)):
        info = s.info.get(info_topic)
        if not s.has(image_topic) or not info:
            print(f"{label} camera: not in this bag")
            continue
        first = s.reader.decode(next(s.reader.messages(topics=[image_topic])), keep_data=False)
        k = intrinsics_of(info)
        try:
            pose = camera_extrinsic(s.tf, BASE_FRAME, first["header"]["frame_id"], first["header"]["stamp_ns"])
            pose_text = ", ".join(f"{key} {pose[key] + 0.0:.2f}" for key in ("x", "y", "z", "roll", "pitch", "yaw"))
        except LookupError as exc:
            pose_text = f"none ({exc})"
        print(f"{label} camera: {first['width']}x{first['height']} {first['encoding']}, fx {k['fx']:.2f} fy {k['fy']:.2f} "
              f"cx {k['cx']:.2f} cy {k['cy']:.2f}; pose in {BASE_FRAME} (UE cm/deg, relative): {pose_text}")
    if s.has(TRACKED):
        print(f"tracked objects: {len(s.tracked_ids)} ids: {', '.join(s.tracked_ids)}")
    if s.has(RAW):
        print(f"segmentation classes: {', '.join(s.classes)}")
    print(f"joints: {'yes' if s.has(JOINTS) else 'no'}; overlay by default: {s.overlay_mode('auto')}")
    if s.frozen_3d():
        print("WARNING: no odom->base_link in /tf: the tracker's 3D poses are frozen where the robot began while the "
              "camera moves on. Use the bag re-processed with visual odometry (_vo) for 3D markers, or "
              "--positions derived.")
    s.close()
    return 0


def cmd_play(args):
    from ramms_tools.bagplay.player import BagSession, PlayOptions, Player, StreamIds
    from ramms_tools.bagplay.sinks import LiveSinks, RecordingSinks
    options = PlayOptions(rate=args.rate, start_s=args.start, end_s=args.end, loop=args.loop, overlay=args.overlay,
                          positions=args.positions, objects=not args.no_objects, joints=not args.no_joints,
                          joints_hz=args.joints_hz, gripper_closed_rad=args.gripper_closed_rad)
    session = BagSession(args.bag)
    if args.dry_run:
        sinks = RecordingSinks()
        stats = Player(session, sinks, PlayOptions(**{**options.__dict__, "rate": 0.0, "loop": False}),
                       StreamIds()).run()
        _summary(stats, sinks)
        return 0

    from ramms_tools.scripts.ui_bridge_test import BridgeClient, BridgeError, JointRig, JointsUnavailable
    from ramms_tools.streaming import StreamSender
    client = BridgeClient(args.host, args.rc_port, preset=args.preset)
    try:
        client.load_preset()
    except BridgeError as exc:
        print(f"cannot reach the UI's Remote Control preset: {exc}")
        return 1
    ids = _ids_from(client)
    rig = None
    if options.joints:
        try:
            rig = JointRig.locate(client)
        except (JointsUnavailable, BridgeError) as exc:
            print(f"  joints not replayed: {exc}")
            options.joints = False  # nothing to send them to: do not read or count them
    sender = StreamSender(host=args.host, port=args.rmss_port, queue_size=0)
    try:
        sender.connect()
    except OSError as exc:
        print(f"cannot connect to RMSS on {args.host}:{args.rmss_port} ({exc}) -- is the UI playing?")
        return 1
    sinks = LiveSinks(client, sender, rig)
    player = Player(session, sinks, options, ids)
    print(f"playing {args.bag} ({session.reader.duration_s:.1f} s) at {args.rate:g}x; overlay: {player.overlay}; "
          f"3D: {options.positions}; Ctrl-C stops")
    failures = []
    stats = player.stats
    try:  # the UI is put back however this ends: the end, Ctrl-C, or an error
        try:
            stats = player.run(close=False)
        except KeyboardInterrupt:
            print("\nstopped")
        if args.verify:
            failures = _verify(args, player, sinks)
    finally:
        sinks.close()
    _summary(stats, sinks)
    if sinks.errors:
        print(f"errors: {sinks.errors}")
    return 1 if failures or sinks.errors else 0


def _verify(args, player, sinks):
    from ramms_tools.bagplay.verify import check_joints, check_markers, read_editor
    from ramms_tools.editor_exec import EditorExec, EditorExecError
    from ramms_tools.scripts.ui_bridge_test import _reporter
    report, failures = _reporter()
    ran = [0]

    def check(ok, what, detail=""):
        ran[0] += 1
        report(ok, what, detail)
    print("\nverify")
    state = player.stats.last_3d_payload
    if state is None:
        print("  no 3D objects were pushed: nothing to check markers against")
    else:
        sinks.objects(state["payload"])  # the newest 3D set, pushed again so the markers are its own
        sinks.drain()
        try:
            with EditorExec(project=args.editor_project) as editor:
                readback = read_editor(editor)
            depth = intrinsics = None
            if player.options.positions == "derived":
                from ramms_tools.bagplay.player import SCENE_INFO, intrinsics_of
                # The provider keeps the latest depth frame, and the re-push below is derived from THAT one --
                # not the frame of the payload's own stamp, which on a moving robot is a different view.
                depth = player.session.depth_at(player.stats.last_scene_depth_ns or state["stamp_ns"])
                info = player.session.info.get(SCENE_INFO)
                intrinsics = intrinsics_of(info) if info else None
            check_markers(state, readback, check, positions=player.options.positions, depth=depth,
                          intrinsics=intrinsics, image_size=player.session.scene_size)
        except (EditorExecError, RuntimeError) as exc:
            check(False, "read the world markers back through the editor's Python", str(exc))
    sinks.drain()  # the last joint send is final before it is read back
    check_joints(sinks, check)
    if not ran[0]:
        print("no checks ran: nothing 3D was pushed and no joints were replayed")
        return ["nothing checked"]
    print(f"{'FAILED: ' + str(len(failures)) if failures else 'all %d checks passed' % ran[0]}")
    return failures


def _summary(stats, sinks):
    sent = ", ".join(f"{k} {v}" for k, v in sorted(stats.sent.items())) or "nothing"
    print(f"sent: {sent}; late frames dropped: {stats.late_dropped}; passes: {stats.passes}")


def main(argv=None):
    from ramms_tools.bagplay.player import OVERLAY_MODES
    parser = argparse.ArgumentParser(prog="ramms-bag", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("info", help="what the bag holds, and what a replay of it will and will not show")
    p.add_argument("bag")
    p = sub.add_parser("play", help="replay the bag into a running RammpUI")
    p.add_argument("bag")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--rmss-port", type=int, default=30030)
    p.add_argument("--rc-port", type=int, default=30010, help="Remote Control HTTP port")
    p.add_argument("--preset", default=None, help="the bridge's Remote Control preset (default: discovered)")
    p.add_argument("--rate", type=float, default=1.0, help="playback speed (0 = as fast as possible)")
    p.add_argument("--start", type=float, default=0.0, help="seconds from the bag's start")
    p.add_argument("--end", type=float, default=None, help="seconds from the bag's start")
    p.add_argument("--loop", action="store_true")
    p.add_argument("--overlay", choices=OVERLAY_MODES, default="auto",
                   help="what the overlay camera slot shows: the wrist camera (default when the bag has one), the "
                        "segmenter's masks over the scene frame, the segmenter's own rendering (needs Pillow), or none")
    p.add_argument("--positions", choices=("sent", "derived"), default="sent",
                   help="sent: the tracker's 3D poses; derived: 2D only, as today's bridge sends, so RammsUI derives "
                        "the markers from depth")
    p.add_argument("--no-objects", action="store_true")
    p.add_argument("--no-joints", action="store_true")
    p.add_argument("--joints-hz", type=float, default=10.0)
    p.add_argument("--gripper-closed-rad", type=float, default=0.793,
                   help="the gripper knuckle angle that means closed (this robot's peaks at 0.793)")
    p.add_argument("--verify", action="store_true",
                   help="afterwards, check the 3D markers landed where the bag says (editor PIE, via its Python)")
    p.add_argument("--editor-project", default="RammpUI")
    p.add_argument("--dry-run", action="store_true", help="send nothing; print what would be sent")
    args = parser.parse_args(argv)
    if getattr(args, "preset", None) is None and args.command == "play":
        from ramms_tools.scripts.ui_bridge_test import PRESET
        args.preset = PRESET
    from ramms_tools.rosbag import BagError
    try:
        return {"info": cmd_info, "play": cmd_play}[args.command](args)
    except (BagError, ValueError) as exc:
        print(f"ramms-bag: {exc}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
