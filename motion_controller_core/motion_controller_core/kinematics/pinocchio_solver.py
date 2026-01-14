import pinocchio as pin
import numpy as np
import sys

class PinocchioSolver:
    def __init__(self):
        self.model = None
        self.data = None
        self.tip_frame_id = None
        self.oMbase_inv = pin.SE3.Identity()
        self.q_neutral = None

    def init(self, urdf_content: str, base_link: str, tip_link: str) -> bool:
        """
        Initialize the solver with URDF content and chain limits.
        
        Args:
            urdf_content: XML string of the URDF.
            base_link: Name of the base link.
            tip_link: Name of the tip link.
            
        Returns:
            bool: True if initialization successful, False otherwise.
        """
        try:
            # 1. Build Full Model from URDF
            full_model = pin.buildModelFromXML(urdf_content)
            
            if not full_model.existFrame(tip_link):
                print(f"[PinocchioSolver] Tip link {tip_link} not found.", file=sys.stderr)
                return False
                
            full_tip_id = full_model.getFrameId(tip_link)
            tip_joint_id = full_model.frames[full_tip_id].parentJoint
            
            # 2. Identify Chain Joints (Tip -> Base)
            base_joint_id = 0 # Default to Universe
            if base_link != "universe" and base_link != "world" and full_model.existFrame(base_link):
                base_joint_id = full_model.frames[full_model.getFrameId(base_link)].parentJoint

            chain_joints = []
            current_joint = tip_joint_id
            
            # Trace up
            while current_joint > base_joint_id:
                chain_joints.append(current_joint)
                current_joint = full_model.parents[current_joint]
                
            # 3. Identify joints to LOCK (all joints NOT in chain)
            joints_to_lock = []
            is_chain_joint = [False] * (full_model.njoints)
            for j in chain_joints:
                is_chain_joint[j] = True
                
            for i in range(1, full_model.njoints):
                if not is_chain_joint[i]:
                    joints_to_lock.append(i)
            
            q_ref = pin.neutral(full_model)
            
            # 4. Build Reduced Model
            if joints_to_lock:
                self.model = pin.buildReducedModel(full_model, joints_to_lock, q_ref)
            else:
                self.model = full_model
                
            self.data = self.model.createData()
            self.q_neutral = pin.neutral(self.model)

            # Update tip frame ID in Reduced Model
            if self.model.existFrame(tip_link):
                self.tip_frame_id = self.model.getFrameId(tip_link)
            else:
                print("[PinocchioSolver] Tip link lost in reduced model.", file=sys.stderr)
                return False

            # Compute oMbase in Reduced Model
            pin.forwardKinematics(self.model, self.data, self.q_neutral)
            pin.updateFramePlacements(self.model, self.data)
            
            if self.model.existFrame(base_link):
                base_frame_id = self.model.getFrameId(base_link)
                self.oMbase_inv = self.data.oMf[base_frame_id].inverse()
            else:
                self.oMbase_inv = pin.SE3.Identity()
                
            return True
            
        except Exception as e:
            print(f"[PinocchioSolver] Exception: {e}", file=sys.stderr)
            return False

    def solve_fk(self, q: np.ndarray) -> tuple[bool, pin.SE3]:
        """
        Compute Forward Kinematics.
        
        Args:
            q: Joint configuration.
            
        Returns:
            (success, pose): boolean success flag and pinocchio.SE3 pose of tip w.r.t base.
        """
        if self.model is None:
            return False, pin.SE3.Identity()
            
        if q.size != self.model.nq:
            print(f"[PinocchioSolver] Joint size mismatch: {q.size} vs {self.model.nq}", file=sys.stderr)
            return False, pin.SE3.Identity()

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        oMtip = self.data.oMf[self.tip_frame_id]
        
        # baseMtip = oMbase^-1 * oMtip
        baseMtip = self.oMbase_inv * oMtip
        
        return True, baseMtip

    def solve_ik(self, target_pose: pin.SE3, q_init: np.ndarray) -> tuple[bool, np.ndarray]:
        """
        Compute Inverse Kinematics using CLIK (Newton-Raphson).
        
        Args:
            target_pose: Desired pose of tip w.r.t base.
            q_init: Initial joint configuration.
            
        Returns:
            (success, q_out): boolean success flag and resulting joint configuration.
        """
        if self.model is None:
            return False, q_init
            
        if q_init.size != self.model.nq:
            print(f"[PinocchioSolver] Joint size mismatch: {q_init.size} vs {self.model.nq}", file=sys.stderr)
            return False, q_init

        q = q_init.copy()
        eps = 1e-4
        max_iter = 1000
        dt = 0.1
        damp = 1e-12
        
        # Transform target pose to world frame for solver
        # target_pose is baseMtarget. We usually need oMtarget.
        # oMtarget = oMbase * baseMtarget
        # But we previously defined oMbase_inv. So oMbase = oMbase_inv.inverse()
        oMtarget = self.oMbase_inv.inverse() * target_pose
        
        for i in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            
            current_pose = self.data.oMf[self.tip_frame_id]
            
            # Error in Local Frame
            err = pin.log(current_pose.inverse() * oMtarget).vector
            
            if np.linalg.norm(err) < eps:
                return True, q
                
            # Compute Jacobian in Local Frame
            J = pin.computeFrameJacobian(self.model, self.data, q, self.tip_frame_id, pin.ReferenceFrame.LOCAL)
            
            # Jacobian correction for SE3 log mismatch (handled roughly by J itself for small errors, 
            # but formally Jlog6 is distinct. For standard CLIK with pinocchio, simple J usually works for small steps.
            # Mirroring C++ implementation which used Jlog6 explicitly)
            
            # C++ was: J_ = -Jlog * J_
            # In Python pinocchio bindings, Jlog6 is available
            Jlog = pin.Jlog6(current_pose.inverse() * oMtarget)
            
            # Note: C++ error definition was log6(current^-1 * target)
            # Jlog maps variation of transform to variation of log6.
            # d(log(M))/dt = Jlog * dM/dt (body velocity)
            # We need v such that J*v = err using Newton.
            # v = pinv(J) * err.
            # Using Jlog: J_effective = Jlog * J. 
            
            J_effective = Jlog @ J
            
            # Damped Pseudo Inverse
            # Jt * (J*Jt + damp*I)^-1 * err
            
            JJt = J_effective @ J_effective.T
            JJt[np.diag_indices_from(JJt)] += damp
            
            v = J_effective.T @ np.linalg.solve(JJt, err)
            
            q = pin.integrate(self.model, q, v * dt)
            
        return False, q
