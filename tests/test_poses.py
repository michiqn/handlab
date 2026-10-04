"""Pose/clip list operations, headless — focus on drag-to-reorder (reorderPoses).

The slots read/write `self._poses` and call `self._save_poses()`; we drive them on a bare object
with a stubbed save (no disk, no Qt signal), the same idiom as test_teleop.
"""
import json
import types

from handlab.bridge_poses import BridgePosesMixin as P


def _fake(poses):
    f = types.SimpleNamespace()
    f._poses = list(poses)
    f._saves = 0
    def _save():
        f._saves += 1
    f._save_poses = _save
    f._find_pose = P._find_pose.__get__(f, P)      # bind the real lookup helper to the fake
    return f


def _names(f):
    return [p["name"] for p in f._poses]


def test_reorder_matches_given_order_and_saves():
    f = _fake([{"name": "a", "kind": "pose"}, {"name": "b", "kind": "clip"}, {"name": "c", "kind": "pose"}])
    P.reorderPoses(f, json.dumps(["c", "a", "b"]))
    assert _names(f) == ["c", "a", "b"]
    assert f._saves == 1
    # the moved objects are the SAME dicts (identity preserved, not rebuilt)
    assert f._poses[0]["kind"] == "pose" and f._poses[1]["kind"] == "pose"


def test_reorder_same_order_is_noop_no_save():
    f = _fake([{"name": "a"}, {"name": "b"}])
    P.reorderPoses(f, json.dumps(["a", "b"]))
    assert _names(f) == ["a", "b"] and f._saves == 0


def test_reorder_is_tolerant_unknown_dropped_missing_appended():
    f = _fake([{"name": "a"}, {"name": "b"}, {"name": "c"}])
    # only 'b' listed (+ a ghost) → 'b' first, the unlisted a/c keep their order after it
    P.reorderPoses(f, json.dumps(["b", "ghost"]))
    assert _names(f) == ["b", "a", "c"]
    assert len(f._poses) == 3                      # never loses an entry


def test_reorder_garbage_is_ignored():
    f = _fake([{"name": "a"}, {"name": "b"}])
    P.reorderPoses(f, "not json at all")
    assert _names(f) == ["a", "b"] and f._saves == 0


def test_delete_and_rename_keep_order():
    f = _fake([{"name": "a", "kind": "pose"}, {"name": "b", "kind": "pose"}, {"name": "c", "kind": "pose"}])
    P.renamePose(f, "b", "beta")
    assert _names(f) == ["a", "beta", "c"]          # rename in place, order unchanged
    P.deletePose(f, "a")
    assert _names(f) == ["beta", "c"]
