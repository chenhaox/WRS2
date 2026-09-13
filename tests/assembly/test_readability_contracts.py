"""Public behavior and lifecycle contracts that must survive internal refactoring."""

from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "examples" / "assembly"))
from _shared.stability_cases import make_case, case_supports
from _shared.robot import make_demo
from wrs.assembly import QualityTransition, StepQuality, quality_depth_first
from wrs.assembly.execution import validate_execution
from wrs.assembly.model import digest
from wrs.assembly.sequence import (
    AuxiliarySupport,
    SequenceConfig,
    SequenceEvaluator,
    plan_sequence,
)


class PlannerContractTests(unittest.TestCase):
    def test_removal_alias_accepts_legacy_three_argument_evaluator(self):
        assembly, state, _ = make_case("stack")
        evaluator = SequenceEvaluator(assembly, SequenceConfig())

        def legacy_evaluate(assembled_state, support_ids, part_id):
            return assembled_state, support_ids, part_id

        with patch.object(evaluator, "evaluate", side_effect=legacy_evaluate):
            received_state, supports, part_id = evaluator.evaluate_removal(state, (), "upper")
        self.assertIs(received_state, state)
        self.assertEqual((supports, part_id), ((), "upper"))

    def test_dfs_is_lazy_and_rejected_complete_plan_never_becomes_incumbent(self):
        requested = []
        checked = []

        def expand(order, part_id):
            order = order or ()
            requested.append(order + (part_id,))
            yield QualityTransition(order + (part_id,), StepQuality(1, 1, 1), part_id)

        def accept_complete(order):
            checked.append(order)
            return order != ("a", "b", "c")

        result = quality_depth_first("abc", expand, accept_complete=accept_complete)
        # Follow the first branch before asking for its siblings' physics.
        self.assertEqual(requested[:3], [("a",), ("a", "b"), ("a", "b", "c")])
        self.assertEqual(checked[:2], [("a", "b", "c"), ("a", "c", "b")])
        self.assertEqual(result["best"][0], ("a", "c", "b"))
        self.assertEqual(result["score"], 1)
        self.assertTrue(result["search_complete"])
        self.assertGreater(result["pruned"], 0)

    def test_insertion_alias_preserves_support_handover_and_cached_geometry(self):
        assembly, state, _ = make_case("floating")
        supports = tuple(
            AuxiliarySupport(f"aux_{index}", candidate)
            for index, candidate in enumerate(case_supports("floating"))
        )
        config = SequenceConfig()
        evaluator = SequenceEvaluator(assembly, config, supports)
        plan = plan_sequence(
            assembly,
            state,
            supports=supports,
            initial_support_ids=("left", "right"),
            config=config,
            evaluator=evaluator,
        )
        self.assertEqual(plan.status, "success")
        expected = plan.assembly_steps[0]
        graph = evaluator.graph(expected.assembly_after)
        graph_count = len(evaluator.graphs)
        balance_count = len(evaluator.balances)
        step, failure = evaluator.evaluate_insertion(
            expected.assembly_after,
            expected.part_id,
            supports_before=expected.assembly_supports_before,
            supports_after=expected.assembly_supports_after,
        )
        self.assertIsNone(failure)
        self.assertIs(step.assembly_before, step.after)
        self.assertIs(step.assembly_after, step.before)
        self.assertEqual(digest(step.assembly_before), digest(expected.assembly_before))
        self.assertEqual(step.assembly_supports_before, expected.assembly_supports_before)
        self.assertEqual(step.assembly_supports_after, expected.assembly_supports_after)
        self.assertNotEqual(step.assembly_supports_before, step.assembly_supports_after)
        self.assertEqual(digest(step.events), digest(expected.events))
        np.testing.assert_array_equal(step.assembly_poses, expected.assembly_poses)
        self.assertIs(evaluator.graph(step.assembly_after), graph)
        self.assertEqual(
            (len(evaluator.graphs), len(evaluator.balances)), (graph_count, balance_count)
        )

    def test_robot_exception_restores_rng_workcell_and_collision_settings(self):
        assembly, cell = make_demo()
        plan = plan_sequence(assembly)
        saved_settings = {}
        for resource_id, binding in cell.bindings.items():
            arm = binding.arm
            saved_settings[resource_id] = (
                arm.collision_cache_size,
                arm.collision_cache_decimals,
                arm.cd_step_size,
            )
        saved_rng = np.random.get_state()
        with patch(
            "wrs.assembly.execution._PolicyCollider.is_collided",
            side_effect=RuntimeError("injected collision-query failure"),
        ):
            with self.assertRaisesRegex(RuntimeError, "injected collision-query failure"):
                validate_execution(plan, cell)
        current_rng = np.random.get_state()
        self.assertEqual(current_rng[0], saved_rng[0])
        np.testing.assert_array_equal(current_rng[1], saved_rng[1])
        self.assertEqual(current_rng[2:], saved_rng[2:])
        for resource_id, binding in cell.bindings.items():
            arm = binding.arm
            self.assertEqual(
                (arm.collision_cache_size, arm.collision_cache_decimals, arm.cd_step_size),
                saved_settings[resource_id],
            )
            np.testing.assert_array_equal(arm.body.qs, cell._initial_q[resource_id])
        for part_id, obj in cell.objects.items():
            np.testing.assert_allclose(obj.tf, cell.initial_poses[part_id], atol=1e-7, rtol=0)


if __name__ == "__main__":
    unittest.main()
