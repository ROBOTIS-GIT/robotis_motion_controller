import numpy as np
import sys
import jax
import jax.numpy as jnp
import jax_dataclasses as jdc
import jaxlie
import jaxls
import pyroki as pk
import yourdfpy
import tempfile
import os

class PyrokiSolver:
    def __init__(self):
        self.robot = None
        self.target_link_indices = None
        self.jitted_solve = None

    def init(self, urdf_content: str, base_link: str, tip_links: list[str] | str) -> bool:
        """
        Initialize the solver with URDF content.
        
        Args:
            urdf_content: XML string of the URDF.
            base_link: Name of the base link.
            tip_links: Name(s) of the tip link(s) to control.
            
        Returns:
            bool: True if initialization successful.
        """
        try:
            # Load URDF via temp file for yourdfpy compatibility
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp:
                tmp.write(urdf_content)
                tmp_path = tmp.name
                
            try:
                urdf_model = yourdfpy.URDF.load(tmp_path)
            finally:
                os.remove(tmp_path)

            self.robot = pk.Robot.from_urdf(urdf_model)
            
            if isinstance(tip_links, str):
                tip_links = [tip_links]
                
            self.target_link_indices = []
            for link in tip_links:
                if link not in self.robot.links.names:
                    print(f"[PyrokiSolver] Tip link {link} not found.", file=sys.stderr)
                    return False
                self.target_link_indices.append(self.robot.links.names.index(link))
            
            self._jit_solver()
            return True
            
        except Exception as e:
            print(f"[PyrokiSolver] Init Exception: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()
            return False

    def _jit_solver(self):
        """Pre-jit the JAX solver function based on pyroki_snippets logic."""
        
        @jdc.jit
        def _solve_ik_jax(
            robot: pk.Robot,
            target_wxyzs: jax.Array,
            target_positions: jax.Array,
            target_link_indices: jax.Array,
            q_init: jax.Array
        ) -> jax.Array:
            # Adapted from _solve_ik_with_multiple_targets.py
            
            JointVar = robot.joint_var_cls
            
            # Construct target poses
            # target_wxyzs: (N, 4), target_positions: (N, 3)
            # jaxlie.SE3 expects batch dimensions if provided.
            # We want to form a batch of N SE3 objects? No, pose_cost_analytic_jac expects ONE target pose usually?
            # Or does it handle multiple?
            # The snippet maps robot: jax.tree.map(lambda x: x[None], robot)
            # and passes target_pose with batch_axes.
            # This implies it solves N problems in parallel OR it treats N targets as one problem?
            # Wait, the snippet says:
            # "Batch axes for the variables and cost terms (e.g., target pose) should be broadcastable!"
            # If we want 1 robot to reach N targets simultaneously, we need N cost terms added together.
            # BUT the snippet uses ONE pose_cost_analytic_jac call.
            # This implies `pose_cost_analytic_jac` handles broadcasting if we give it correct shapes?
            # Let's look at the snippet implementation of costs list:
            # costs = [ pk.costs.pose_cost_analytic_jac(..., target_joint_indices, ...) ]
            # If target_joint_indices is an array, maybe it sums costs?
            # Actually, `pose_cost_analytic_jac` documentation/code isn't fully visible, but based on usage:
            # "target_pose = ... with batch_axes"
            # It seems the snippet sets up a Batched Problem where we have N targets.
            
            # Use specific logic for "Multiple Targets for One Configuration" vs "Multiple Configurations".
            # The snippet `_solve_ik_with_multiple_targets` seems to do:
            # Solve for ONE cfg that satisfies N targets?
            # It returns `sol[JointVar(0)]`. This implies one configuration variable.
            # So `pose_cost_analytic_jac` likely sums costs if inputs are batched?
            # OR we need to add multiple cost terms.
            
            # Re-reading snippet:
            # It does: `target_pose = ...` (Batch size N)
            # `robot` is mapped to `x[None]` (Batch size 1?)
            # `JointVar(jnp.full(batch_axes, 0))` (Batch size N)
            # It seems this might be setting up N *independent* problems if batch_axes > 0?
            # BUT the return is `sol[JointVar(0)]`.
            
            # If we want 1 robot to meet multiple targets (e.g. 2 hands), we usually simply sum the residuals.
            # I will follow the snippet exactly, but carefully.
            # "Get the batch axes... target_pose.get_batch_axes()"
            # If N=2, batch_axes=(2,)
            # robot mapped to x[None] -> (1, ...)
            # JointVar mapped to (2, 0).
            # This looks like it solves 2 separate problems?
            # Wait, `solve_ik_with_multiple_targets` docstring says returns `cfg` shape `(actuated_count,)`.
            # If it solved 2 problems, it would return (2, actuated_count).
            # The Example 02_bimanual_ik passes 2 targets.
            
            # Let's assume the snippet logic works for bimanual on a single robot.
            # It creates `target_pose` with shape (N,).
            # It creates `factors` (costs) using this target_pose.
            # `jaxls` creates a problem.
            
            # Important: We need q_init support.
            # `initial_values={JointVar(0): q_init}`
            
            target_pose = jaxlie.SE3.from_rotation_and_translation(
                jaxlie.SO3(target_wxyzs), target_positions
            )
            # If we have 1 target, shape is (). If N targets, (N,).
            
            # batch_axes?
            # If we pass multiple targets (N, ...), target_pose has batch format.
            batch_axes = target_pose.get_batch_axes()
            
            # Note: The snippet does `jax.tree.map(lambda x: x[None], robot)`.
            # This adds a batch dim to robot.
            # `JointVar(jnp.full(batch_axes, 0))` creates variables for *each* batch element?
            # If batch_axes=(2,), we have 2 variables?
            # If we have 2 variables, do they represent the *same* physical robot joints?
            # Yes if we are just solving for "robot configuration".
            # BUT if jaxls treats them as batch, it solves them independently?
            # If independent, we get 2 results.
            
            # Let's look at `_solve_ik_with_multiple_targets` return: `assert cfg.shape == (robot.joints.num_actuated_joints,)`.
            # This asserts a SINGLE configuration is returned.
            # So `_solve_ik_jax` returns a single array.
            
            # This implies the snippet's `_solve_ik_jax` handles the reduction?
            # Or `jaxls` solves for one variable that minimizes the sum of costs for all targets?
            # But the snippet passes `JointVar(jnp.full(batch_axes, 0))`.
            # If batch_axes is (2,), then Variable has batch shape (2,).
            # Which suggests 2 independent problems.
            
            # Alternative: Bimanual IK (one robot, two hands) usually IS just one problem with 2 costs.
            # We should probably add 2 separate cost terms for the SAME variable `JointVar(0)`.
            
            # Let's diverge slightly from the snippet if the snippet is for batched-independent IK (like finding solutions for multiple unrelated targets).
            # Example 02 says "Same as 01... but with two end effectors".
            # And it uses `solve_ik_with_multiple_targets`.
            
            # I will trust the user wants code "like pyroki_snippets".
            # I will implement it almost verbatim, but I need to handle `q_init` correctly.
            
            JointVar = robot.joint_var_cls
            target_pose = jaxlie.SE3.from_rotation_and_translation(
                jaxlie.SO3(target_wxyzs), target_positions
            )
            batch_axes = target_pose.get_batch_axes()
            # If batch_axes is empty (single target), it works.
            # If batch_axes is (N,), we might be broadcasting.
            
            costs = [
                pk.costs.pose_cost_analytic_jac(
                    jax.tree.map(lambda x: x[None], robot), # Batched robot?
                    JointVar(jnp.full(batch_axes, 0)),      # Batched variable?
                    target_pose,
                    target_link_indices,
                    pos_weight=50.0,
                    ori_weight=10.0,
                ),
                pk.costs.rest_cost(
                    JointVar(0), # Single variable?
                    rest_pose=JointVar.default_factory(),
                    weight=1.0,
                ),
            ]
            costs.append(
                 pk.costs.limit_constraint(
                    robot,
                    JointVar(0),
                )
            )
            
            # CAUTION: Mixing Batched and Non-Batched variables in factors/costs might be tricky in jaxls.
            # If `pose_cost` uses `JointVar(batch)` and `rest_cost` uses `JointVar(0)`, are they the same variable?
            # `jaxls` uses the ID (0 in this case).
            # `jnp.full(batch_axes, 0)` makes an array of IDs? No, `JointVar` expects integer ID.
            # Ah, `JointVar` is `jaxls.Var`. `jaxls.Var(id)`.
            # `jaxls.Var` does not support batching of IDs usually.
            # Wait, `jnp.full(batch_axes, 0)` creates an array of zeros.
            # If `JointVar` is a dataclass, passing an array to it ...
            
            # Let's simplify. I will add N separate pose costs for the SAME Single Variable ID 0.
            # This guarantees we solve for ONE robot configuration that satisfies ALL targets.
            
            costs = []
            
            # Rest cost
            costs.append(pk.costs.rest_cost(JointVar(0), weight=1.0, rest_pose=q_init)) 
            # Limit cost
            costs.append(pk.costs.limit_constraint(robot, JointVar(0)))
            
            # Flatten targets if necessary
            # We iterate over targets and add a cost for each.
            
            # Because we are inside JIT, we can't iterate if number of targets is dynamic/variable.
            # However, `init` fixed the number of tip links. `target_link_indices` is known at init?
            # But here `target_link_indices` is passed as argument.
            # If we JIT this function, `target_link_indices` should be static or fixed size?
            # The snippet uses `target_joint_indices` as JAX Array.
            
            # Function: One pose cost with batched inputs?
            # `pose_cost_analytic_jac` signature:
            # (robot, var, target_pose, target_link_index, ...)
            # if we pass Batched target_pose and Batched target_link_index, does it sum?
            # `jaxls` sums residuals squared.
            # If `pose_cost` returns a batched residual, `jaxls` will minimize the norm of that batch.
            # That is exactly what we want: sum of squares of errors for all targets.
            
            # So, we pass arrays!
            # Robot needs to be broadcast compatible?
            # If we have N targets, and 1 robot, we can map robot to have dim 1 or replicate.
            # The snippet uses `jax.tree.map(lambda x: x[None], robot)`. equivalent to adding dimension (1, ...).
            # That broadcasts with (N, ...).
            
            # Variable: We want ONE solution (1, ...).
            # Snippet uses `JointVar(jnp.full(batch_axes, 0))`.
            # If batch_axes is (N,), this creates `JointVar` containing an array of zeros?
            # Does `jaxls` interpret this as "Use Variable 0 for all batch elements"?
            # This seems to be the way.
            
            costs.append(
                pk.costs.pose_cost_analytic_jac(
                    jax.tree.map(lambda x: x[None], robot),
                    JointVar(jnp.full(batch_axes, 0)), # Use Var 0 for all targets
                    target_pose,
                    target_link_indices,
                    pos_weight=50.0,
                    ori_weight=10.0,
                )
            )
            
            problem = jaxls.LeastSquaresProblem(costs=costs, variables=[JointVar(0)])
            sol = problem.analyze().solve(
                verbose=False,
                linear_solver="dense_cholesky",
                trust_region=jaxls.TrustRegionConfig(lambda_initial=10.0),
            )
            return sol[JointVar(0)]
            
        self.jitted_solve = _solve_ik_jax

    def solve_ik(self, target_poses: list[np.ndarray] | np.ndarray, q_init: np.ndarray) -> tuple[bool, np.ndarray]:
        """
        Solve IK for one or multiple targets.
        
        Args:
            target_poses: List of 4x4 matrices or single 4x4 matrix.
            q_init: Initial config.
            
        Returns:
            (success, q_out)
        """
        if self.robot is None or self.jitted_solve is None:
            return False, q_init

        try:
            # Normalize target_poses to list/array
            if isinstance(target_poses, np.ndarray) and target_poses.shape == (4,4):
                target_poses = [target_poses]
                
            num_targets = len(target_poses)
            if num_targets != len(self.target_link_indices):
                print(f"Mismatch targets {num_targets} vs links {len(self.target_link_indices)}", file=sys.stderr)
                return False, q_init
                
            # Extract wxyz, pos
            wxyzs = []
            positions = []
            for T in target_poses:
                T_se3 = jaxlie.SE3.from_matrix(T)
                wxyzs.append(T_se3.rotation().wxyz)
                positions.append(T_se3.translation())
                
            target_wxyzs = jnp.array(wxyzs) # Shape (N, 4)
            target_positions = jnp.array(positions) # Shape (N, 3)
            target_indices = jnp.array(self.target_link_indices) # Shape (N,)
            
            q_out = self.jitted_solve(
                self.robot,
                target_wxyzs,
                target_positions,
                target_indices,
                jnp.array(q_init)
            )
            
            # If shape is (1, DoF), flatten?
            q_out = np.array(q_out)
            if q_out.ndim > 1:
                q_out = q_out.flatten()
                
            return True, q_out
            
        except Exception as e:
            print(f"[PyrokiSolver] IK Failed: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()
            return False, q_init

    def solve_fk(self, q: np.ndarray) -> tuple[bool, list[np.ndarray]]:
        """
        Solve FK for initialized tip links.
        Returns list of 4x4 matrices.
        """
        if self.robot is None: return False, []
        try:
            link_poses = self.robot.forward_kinematics(jnp.array(q))
            
            results = []
            for idx in self.target_link_indices:
                pose_7 = link_poses[idx]
                results.append(np.array(jaxlie.SE3(pose_7).as_matrix()))
                
            return True, results
        except Exception as e:
            print(f"[PyrokiSolver] FK Failed: {e}", file=sys.stderr)
            return False, []
