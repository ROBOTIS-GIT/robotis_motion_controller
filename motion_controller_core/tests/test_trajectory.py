import pytest
import numpy as np
import matplotlib.pyplot as plt
from motion_controller_core.trajectory.scurve_profile import SCurveProfile

def test_simple_profile():
    # Start: 0, End: 10, V: 2, A: 1, J: 10
    sc = SCurveProfile(0.0, 10.0, 2.0, 1.0, 10.0)
    sc.compute()
    
    assert sc.total_duration > 0
    p, v, a = sc.evaluate(sc.total_duration)
    assert np.isclose(p, 10.0, atol=1e-3)
    assert np.isclose(v, 0.0, atol=1e-3)
    assert np.isclose(a, 0.0, atol=1e-3)

def test_limits_adherence():
    # Start: 0, End: 5, V: 1, A: 2, J: 5
    v_max = 1.0
    a_max = 2.0
    j_max = 5.0
    sc = SCurveProfile(0.0, 5.0, v_max, a_max, j_max)
    sc.compute()
    
    dt = 0.01
    times = np.arange(0, sc.total_duration + dt, dt)
    
    for t in times:
        p, v, a = sc.evaluate(t)
        # Allow tiny epsilon for float error
        assert abs(v) <= v_max + 1e-4
        assert abs(a) <= a_max + 1e-4
        
        # Jerk is not directly returned but we know it's step function.
        # We could check jerk consistency if we differentiated 'a', but strictly the class doesn't output J.
        
    # Check End
    p_end, _, _ = sc.evaluate(sc.total_duration)
    assert np.isclose(p_end, 5.0, atol=1e-3)

    # Visualization
    plot_profile(sc, "test_limits_adherence.png")

def plot_profile(sc, filename):
    dt = 0.005
    times = np.arange(0, sc.total_duration + dt, dt)
    ps, vs, as_ = [], [], []
    
    for t in times:
        p, v, a = sc.evaluate(t)
        ps.append(p)
        vs.append(v)
        as_.append(a)
        
    plt.figure(figsize=(10, 8))
    
    plt.subplot(3, 1, 1)
    plt.plot(times, ps)
    plt.title(f"Position (Target: {sc.p1})")
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.plot(times, vs)
    plt.title(f"Velocity (Max: {sc.v_max})")
    plt.grid(True)
    
    plt.subplot(3, 1, 3)
    plt.plot(times, as_)
    plt.title(f"Acceleration (Max: {sc.a_max})")
    plt.grid(True)
    
    plt.tight_layout()
    try:
        # Try to show if display is available, otherwise save
        if plt.get_backend() != 'agg':
             plt.show(block=False)
             plt.pause(1)
        plt.savefig(filename)
        print(f"Saved plot to {filename}")
    except Exception as e:
        print(f"Failed to visualize: {e}")
    plt.close()

def test_short_move():
    # Very short move, never reaches v_max
    sc = SCurveProfile(0.0, 0.01, 10.0, 10.0, 100.0)
    sc.compute()
    
    p_end, v_end, a_end = sc.evaluate(sc.total_duration)
    assert np.isclose(p_end, 0.01, atol=1e-4)
    assert np.isclose(v_end, 0.0, atol=1e-4)

def test_negative_move():
    sc = SCurveProfile(10.0, 0.0, 2.0, 1.0, 5.0)
    sc.compute()
    
    p_end, v_end, a_end = sc.evaluate(sc.total_duration)
    assert np.isclose(p_end, 0.0, atol=1e-3)
    
    _, v_mid, _ = sc.evaluate(sc.total_duration/2)
    assert v_mid < 0

if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__]))
