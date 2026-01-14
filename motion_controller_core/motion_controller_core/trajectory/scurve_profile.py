import numpy as np
import math

class SCurveProfile:
    def __init__(self, start_pos, end_pos, max_vel, max_acc, max_jerk):
        """
        Initialize S-Curve Profile Generator.
        
        Args:
            start_pos (float): Starting position
            end_pos (float): Target position
            max_vel (float): Maximum velocity
            max_acc (float): Maximum acceleration
            max_jerk (float): Maximum jerk
        """
        self.p0 = start_pos
        self.p1 = end_pos
        self.v_max = abs(max_vel)
        self.a_max = abs(max_acc)
        self.j_max = abs(max_jerk)
        
        self.total_duration = 0.0
        self.times = []
        self.jerks = []
        self.computed = False
        
        # Direction
        self.direction = 1.0 if end_pos >= start_pos else -1.0
        self.distance = abs(end_pos - start_pos)

    def compute(self):
        """
        Compute the S-Curve profile segments.
        Based on standard 7-segment jerk-limited profile logic.
        """
        if self.distance < 1e-9:
            self.total_duration = 0.0
            self.times = [0.0] * 7
            self.jerks = [0.0] * 7
            self.computed = True
            return True

        # 1. Determine kinematic limits reachable
        # Check if v_max is reachable given a_max and j_max
        # V_a = a_max^2 / j_max (velocity reached when accelerating to a_max and back to 0)
        
        # If v_max is very low, we might not reach a_max
        if self.v_max * self.j_max < self.a_max**2:
            # We are velocity limited during acceleration phase.
            # Max acceleration reached will be a_peak = sqrt(v_max * j_max)
            # t1 = a_peak / j_max = sqrt(v_max/j_max)
            # t2 = 0
            pass 
        
        # Iterative approach or Analytic?
        # Let's use analytic logic for standard Double S-Curve.
        
        v = self.v_max
        a = self.a_max
        j = self.j_max
        
        # Phase 1: Acceleration to v_max
        # Time to reach a_max
        t1 = a / j
        # Vel change
        if a * t1 > v:
            # We don't reach a_max, we are velocity limited in acceleration
            t1 = math.sqrt(v / j)
            t2 = 0.0
            a_limit = j * t1
        else:
            # We reach a_max
            t1 = a / j
            t2 = (v / a) - t1
            a_limit = a
            
        # Distance during acceleration (and deceleration)
        # d_acc = v_end_of_acc * (2*t1 + t2) / 2
        # Here v_end = v (target)
        d_acc = v * (2*t1 + t2) / 2.0
        
        # Total distance required for full ramp up and down
        d_required = 2 * d_acc
        
        if d_required > self.distance:
            # We don't reach v_max. Distance is the limit.
            # We need to find new v_peak < v_max.
            # a_limit might also change if v_peak is very small.
            
            # Assume t2 = 0 (Triangular profile in Velocity)
            # We check if we can reach a_max with the given distance.
            # Min dist to reach a_max is when v_peak = a_max^2/j
            # d_min_a = 2 * ( (a_max^2/j) * (2*a_max/j)/2 ) = 2 * a_max^3 / j^2
            
            if self.distance > 2 * (a**3 / j**2):
                # We reach a_max, but not v_max. t2 != 0 in acceleration? 
                # No, standard assumption:
                # If d_required > dist, we reduce v.
                # If we reduced v such that we still reach a_max?
                # Yes, if distance is "medium".
                
                # Equation for distance with v_peak, assuming a reached a_max:
                # v_peak = a * (t1 + t2)  where t1 = a/j fixed.
                # d = v_peak * (2*t1 + t2)
                # d = a*(t1+t2) * (2*t1+t2)
                # Solve for t2.
                
                t1 = a / j
                # (t1+t2)*(2t1+t2) = d/a
                # t2^2 + 3t1t2 + 2t1^2 - d/a = 0
                
                delta = 9*t1**2 - 4*(1.0)*(2*t1**2 - self.distance/a)
                if delta < 0: delta = 0
                t2 = (-3*t1 + math.sqrt(delta)) / 2.0
                t4 = 0.0
                
            else:
                # We don't reach a_max. t2 = 0 (Accel profile allows no cruise at max A)
                # t4 = 0 (No cruise at max V)
                # d = 2 * ( j*t1^3 )
                # t1 = cbrt(d / 2j)
                t1 = (self.distance / (2*j))**(1/3.0)
                t2 = 0.0
                t4 = 0.0
                
        else:
            # We reach v_max.
            # Cruise phase exists.
            t4 = (self.distance - d_required) / v
            
        # Construct segments
        # 1: +J (t1) -> Ramp up Accel
        # 2: 0 (t2)  -> Constant Accel
        # 3: -J (t1) -> Ramp down Accel
        # 4: 0 (t4)  -> Constant Vel
        # 5: -J (t1) -> Ramp up Decel
        # 6: 0 (t2)  -> Constant Decel
        # 7: +J (t1) -> Ramp down Decel
        
        self.times = [t1, t2, t1, t4, t1, t2, t1]
        self.jerks = [j, 0.0, -j, 0.0, -j, 0.0, j]
        
        # Apply Direction
        self.jerks = [val * self.direction for val in self.jerks]
        
        self.total_duration = sum(self.times)
        self.computed = True
        return True

    def evaluate(self, t):
        if not self.computed:
            return self.p0, 0.0, 0.0
            
        if t < 0: return self.p0, 0.0, 0.0
        # Be robust for small float errors at end
        if t >= self.total_duration - 1e-9: return self.p1, 0.0, 0.0
        
        p = self.p0
        v = 0.0
        a = 0.0
        
        t_rem = t
        
        for i in range(7):
            dt = self.times[i]
            if dt <= 0: continue # Skip empty segments
            j = self.jerks[i]
            
            if t_rem <= dt:
                # Active segment
                t_eval = t_rem
                p_next = p + v*t_eval + 0.5*a*t_eval**2 + (1.0/6.0)*j*t_eval**3
                v_next = v + a*t_eval + 0.5*j*t_eval**2
                a_next = a + j*t_eval
                return p_next, v_next, a_next
            else:
                # Integrate full segment
                p = p + v*dt + 0.5*a*dt**2 + (1.0/6.0)*j*dt**3
                v = v + a*dt + 0.5*j*dt**2
                a = a + j*dt
                t_rem -= dt
                
        return self.p1, 0.0, 0.0
