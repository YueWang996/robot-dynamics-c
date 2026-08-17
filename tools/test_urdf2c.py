#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
test_urdf2c.py -- tests for the URDF -> rd_model_t converter.

    python3 tools/test_urdf2c.py                    # unit tests only
    python3 tools/test_urdf2c.py --urdf-root PATH   # also cross-checks the real
                                                    # robot URDFs against Pinocchio

The unit tests build small URDFs in a temp dir and assert on the generated C.
The Pinocchio tests are skipped automatically when pinocchio is not installed.

Run with -v for the usual unittest verbosity.
"""

import os
import re
import subprocess
import sys
import tempfile
import textwrap
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import urdf2c  # noqa: E402

URDF_ROOT = None


def write(tmp, name, text):
    path = os.path.join(tmp, name)
    with open(path, "w") as fh:
        fh.write(textwrap.dedent(text))
    return path


def convert(tmp, urdf_path, name="bot", floating=False):
    links = urdf2c.load_urdf(urdf_path)
    order = urdf2c.topo_sort(links)
    text, n_links, n_joints, nv = urdf2c.emit(name, links, order, floating, urdf_path)
    return text, links, order, n_links, n_joints, nv


CHAIN = """\
    <robot name="chain">
      <link name="base"><inertial><mass value="1"/>
        <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
      <link name="a"><inertial><mass value="2"/><origin xyz="0.1 0 0"/>
        <inertia ixx="1" iyy="2" izz="3" ixy="0.1" ixz="0.2" iyz="0.3"/></inertial></link>
      <link name="b"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="a"/>
        <origin xyz="0 0 0.5" rpy="0 0 0.25"/><axis xyz="0 0 1"/>
        <limit lower="-1" upper="2" effort="3" velocity="4"/>
      </joint>
      <joint name="j2" type="prismatic">
        <parent link="a"/><child link="b"/>
        <origin xyz="1 2 3"/><axis xyz="0 -1 0"/>
        <limit lower="-5" upper="6" effort="7" velocity="8"/>
      </joint>
    </robot>
    """


class TestParsing(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def test_topology_and_counts(self):
        p = write(self.tmp, "chain.urdf", CHAIN)
        text, links, order, n_links, n_joints, nv = convert(self.tmp, p)
        self.assertEqual(order, ["base", "a", "b"])
        self.assertEqual((n_links, n_joints, nv), (3, 2, 2))

    def test_floating_base_dof(self):
        p = write(self.tmp, "chain.urdf", CHAIN)
        text, _, _, n_links, n_joints, nv = convert(self.tmp, p, floating=True)
        self.assertEqual(nv, 8)                       # 6 + 2
        self.assertIn("RD_JOINT_FLOATING", text)
        self.assertIn(".use_floating_base = 1", text)

    def test_parent_precedes_child(self):
        """rd_chain_build's fallback ordering depends on this invariant."""
        p = write(self.tmp, "chain.urdf", CHAIN)
        text, _, _, _, _, _ = convert(self.tmp, p)
        parents = [int(m) for m in re.findall(r"\.parent_idx = (-?\d+),", text)]
        for child, parent in enumerate(parents):
            self.assertLess(parent, child, f"link {child} has parent {parent}")

    def test_inertial_values_round_trip(self):
        p = write(self.tmp, "chain.urdf", CHAIN)
        text, _, _, _, _, _ = convert(self.tmp, p)
        block = text.split("[1] a */")[1]
        self.assertIn("RD_REAL(2.0)", block)          # mass
        self.assertIn("RD_REAL(0.1)", block)          # com.x and ixy
        for want in ("Ixx = RD_REAL(1.0)", "Iyy = RD_REAL(2.0)", "Izz = RD_REAL(3.0)",
                     "Ixy = RD_REAL(0.1)", "Ixz = RD_REAL(0.2)", "Iyz = RD_REAL(0.3)"):
            self.assertIn(want, block)

    def test_joint_limits_and_origin(self):
        p = write(self.tmp, "chain.urdf", CHAIN)
        text, _, _, _, _, _ = convert(self.tmp, p)
        a = text.split("[1] a */")[1].split("[2] b */")[0]
        self.assertIn(".pos_parent = {RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.5)}", a)
        self.assertIn(".rpy_parent = {RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.25)}", a)
        self.assertIn(".q_min = RD_REAL(-1.0), .q_max = RD_REAL(2.0)", a)
        self.assertIn(".dq_max = RD_REAL(4.0), .tau_max = RD_REAL(3.0)", a)

    def test_joint_types_and_negative_axis(self):
        p = write(self.tmp, "chain.urdf", CHAIN)
        text, _, _, _, _, _ = convert(self.tmp, p)
        self.assertIn("RD_JOINT_REVOLUTE", text)
        self.assertIn("RD_JOINT_PRISMATIC", text)
        self.assertIn("RD_AXIS_NEG_Y", text)

    def test_continuous_maps_to_revolute(self):
        p = write(self.tmp, "c.urdf", """\
            <robot name="c">
              <link name="base"/><link name="a"/>
              <joint name="j" type="continuous">
                <parent link="base"/><child link="a"/><axis xyz="1 0 0"/>
              </joint>
            </robot>
            """)
        text, _, _, _, n_joints, _ = convert(self.tmp, p)
        self.assertEqual(n_joints, 1)
        self.assertIn("RD_JOINT_REVOLUTE", text)

    def test_fixed_joint_consumes_no_dof(self):
        p = write(self.tmp, "f.urdf", """\
            <robot name="f">
              <link name="base"/><link name="a"/><link name="b"/>
              <joint name="j1" type="fixed">
                <parent link="base"/><child link="a"/><origin xyz="0 0 1"/></joint>
              <joint name="j2" type="revolute">
                <parent link="a"/><child link="b"/><axis xyz="0 1 0"/>
                <limit lower="0" upper="1" effort="1" velocity="1"/></joint>
            </robot>
            """)
        _, _, _, n_links, n_joints, nv = convert(self.tmp, p)
        self.assertEqual((n_links, n_joints, nv), (3, 1, 1))


class TestJointOrdering(unittest.TestCase):
    """
    The generated q ordering must match Pinocchio's, which is depth-first in
    URDF joint-declaration order. Breadth-first passes the parent<child
    invariant too, so only a branched robot catches the difference.
    """

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        # Two legs of two links each, declared leg by leg.
        joints = []
        links = ['<link name="base"/>']
        for leg in ("L", "R"):
            for seg in ("hip", "knee"):
                links.append(f'<link name="{leg}_{seg}"/>')
            joints.append(f"""
              <joint name="{leg}_hip_j" type="revolute">
                <parent link="base"/><child link="{leg}_hip"/><axis xyz="0 1 0"/>
                <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
              <joint name="{leg}_knee_j" type="revolute">
                <parent link="{leg}_hip"/><child link="{leg}_knee"/><axis xyz="0 1 0"/>
                <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>""")
        self.path = write(self.tmp, "legs.urdf",
                          "<robot name='legs'>" + "".join(links) + "".join(joints) + "</robot>")

    def test_depth_first_in_joint_order(self):
        _, _, order, _, _, _ = convert(self.tmp, self.path)
        self.assertEqual(order, ["base", "L_hip", "L_knee", "R_hip", "R_knee"])

    def test_q_index_comment_matches_order(self):
        text, _, _, _, _, _ = convert(self.tmp, self.path)
        listed = re.findall(r"\*\s+q\[(\d+)\]\s+(\S+)\s+->", text)
        self.assertEqual([n for _, n in listed],
                         ["L_hip_j", "L_knee_j", "R_hip_j", "R_knee_j"])
        self.assertEqual([int(i) for i, _ in listed], [0, 1, 2, 3])


class TestRejections(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def test_non_axis_aligned_axis_is_rejected(self):
        p = write(self.tmp, "d.urdf", """\
            <robot name="d">
              <link name="base"/><link name="a"/>
              <joint name="j" type="revolute">
                <parent link="base"/><child link="a"/><axis xyz="1 1 0"/>
                <limit lower="0" upper="1" effort="1" velocity="1"/></joint>
            </robot>
            """)
        with self.assertRaises(ValueError) as ctx:
            convert(self.tmp, p)
        self.assertIn("axis-aligned", str(ctx.exception))

    def test_unsupported_joint_type_is_rejected(self):
        p = write(self.tmp, "p.urdf", """\
            <robot name="p">
              <link name="base"/><link name="a"/>
              <joint name="j" type="planar">
                <parent link="base"/><child link="a"/></joint>
            </robot>
            """)
        with self.assertRaisesRegex(ValueError, "unsupported joint type"):
            convert(self.tmp, p)

    def test_multiple_roots_is_rejected(self):
        p = write(self.tmp, "m.urdf", """\
            <robot name="m"><link name="a"/><link name="b"/></robot>
            """)
        with self.assertRaisesRegex(ValueError, "exactly one root"):
            convert(self.tmp, p)

    def test_normalised_axis_is_accepted(self):
        """A unit axis written as -0.0/1.0 floats must still quantise cleanly."""
        self.assertEqual(urdf2c.quantise_axis((0.0, 0.0, -1.0)), "RD_AXIS_NEG_Z")
        self.assertEqual(urdf2c.quantise_axis((0.0, 2.0, 0.0)), "RD_AXIS_Y")

    def test_zero_axis_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "zero-length"):
            urdf2c.quantise_axis((0.0, 0.0, 0.0))


class TestNameHandling(unittest.TestCase):
    def test_long_names_truncate_and_stay_unique(self):
        used = set()
        a = urdf2c.cname("a_very_long_link_name_one", used)
        b = urdf2c.cname("a_very_long_link_name_two", used)
        self.assertLessEqual(len(a), 15)
        self.assertLessEqual(len(b), 15)
        self.assertNotEqual(a, b, "truncation collided silently")

    def test_names_fit_rd_link_t(self):
        """rd_link_t.name is char[16], so 15 chars plus the NUL."""
        used = set()
        for i in range(30):
            self.assertLessEqual(len(urdf2c.cname("identical_prefix_%d" % i, used)), 15)


class TestAgainstPinocchio(unittest.TestCase):
    """Cross-checks the real robot URDFs. Skipped unless --urdf-root is given."""

    ROBOTS = {
        "spine": ("tests/spine.urdf", True),
        "xarm7": ("examples/example_robots/xarm_description/urdf/xarm7.urdf", False),
        "go2":   ("examples/example_robots/go2_description/urdf/go2.urdf", True),
    }

    @classmethod
    def setUpClass(cls):
        if URDF_ROOT is None:
            raise unittest.SkipTest("pass --urdf-root to enable")
        try:
            import pinocchio  # noqa: F401
        except ImportError:
            raise unittest.SkipTest("pinocchio not installed")

    def test_joint_order_and_dof_match_pinocchio(self):
        import pinocchio as pin
        tmp = tempfile.mkdtemp()
        for robot, (rel, floating) in self.ROBOTS.items():
            with self.subTest(robot=robot):
                urdf = os.path.join(URDF_ROOT, rel)
                text, links, order, n_links, n_joints, nv = convert(
                    tmp, urdf, name=robot, floating=floating)

                model = (pin.buildModelFromUrdf(urdf, pin.JointModelFreeFlyer())
                         if floating else pin.buildModelFromUrdf(urdf))

                self.assertEqual(nv, model.nv, "velocity DOF mismatch")

                ours = [links[n].joint_name for n in order
                        if links[n].joint_type != "RD_JOINT_FIXED"]
                theirs = list(model.names[2:] if floating else model.names[1:])
                self.assertEqual(ours, theirs,
                                 "q ordering differs from Pinocchio -- a q vector "
                                 "would be silently scrambled between the two")

    def test_total_mass_matches_pinocchio(self):
        import pinocchio as pin
        tmp = tempfile.mkdtemp()
        for robot, (rel, floating) in self.ROBOTS.items():
            with self.subTest(robot=robot):
                urdf = os.path.join(URDF_ROOT, rel)
                _, links, order, _, _, _ = convert(tmp, urdf, name=robot,
                                                   floating=floating)
                model = (pin.buildModelFromUrdf(urdf, pin.JointModelFreeFlyer())
                         if floating else pin.buildModelFromUrdf(urdf))
                ours = sum(links[n].mass for n in order)
                # Index 0 is the universe body. Links joined to the world by a
                # fixed joint before the first movable one -- xarm7's link_base,
                # for instance -- have their mass folded in there, so it counts.
                theirs = sum(i.mass for i in model.inertias)
                self.assertAlmostEqual(ours, theirs, places=6)

    def test_generated_header_compiles(self):
        tmp = tempfile.mkdtemp()
        for robot, (rel, floating) in self.ROBOTS.items():
            with self.subTest(robot=robot):
                urdf = os.path.join(URDF_ROOT, rel)
                text, *_ = convert(tmp, urdf, name=robot, floating=floating)
                hdr = os.path.join(tmp, f"model_{robot}.h")
                with open(hdr, "w") as fh:
                    fh.write(text)
                src = os.path.join(tmp, f"use_{robot}.c")
                with open(src, "w") as fh:
                    fh.write(f'#include "model_{robot}.h"\n'
                             f"int main(void) {{ return {robot}_model_get()->num_links; }}\n")
                r = subprocess.run(
                    ["cc", "-std=c11", "-Wall", "-Werror", "-c",
                     "-DRD_MAX_LINKS=40", "-DRD_MAX_JOINTS=24",
                     "-I", os.path.join(os.path.dirname(HERE), "RobotDynamics"),
                     "-I", tmp, src, "-o", os.path.join(tmp, f"{robot}.o")],
                    capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, r.stderr)


if __name__ == "__main__":
    argv = sys.argv[:]
    if "--urdf-root" in argv:
        i = argv.index("--urdf-root")
        URDF_ROOT = argv[i + 1]
        del argv[i:i + 2]
    unittest.main(argv=argv)
