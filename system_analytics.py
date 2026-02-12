import numpy as np
from rl_pilot import AdvancedRLPilot
from entropy_engine import EntropyEngine

class SystemValidator:
    """
    Independent Verification & Validation (IV&V) Module.
    """
    @staticmethod
    def run_monte_carlo(iterations=50):
        results = {"accuracy": [], "fuel": []}
        REQ_THRESHOLD = 98.0 
        murphy = EntropyEngine()
        
        for i in range(iterations):
            pilot = AdvancedRLPilot()
            initial_dist = np.linalg.norm(pilot.state[:3])
            
            # --- PHYSICS LOOP ---
            for _ in range(2500): 
                # 1. Inject Noise
                noisy_state = murphy.inject_noise(pilot.state)
                
                # 2. Pilot Calculation (Pass full state vector)
                thrust = pilot.get_control_effort(noisy_state)
                
                # 3. Physics Updates
                accel = thrust / pilot.mass
                pilot.state[3:] += accel * pilot.dt
                pilot.state[:3] += pilot.state[3:] * pilot.dt
                
                pilot.total_delta_v += (np.linalg.norm(thrust) / pilot.mass) * pilot.dt
                
                if np.linalg.norm(pilot.state[:3]) < 0.05: break
            
            # --- SCORING ---
            final_dist = np.linalg.norm(pilot.state[:3] - pilot.target)
            acc = max(0, (1 - (final_dist / initial_dist)) * 100)
            results["accuracy"].append(acc)
            results["fuel"].append(pilot.total_delta_v)
            
        # --- STATS ---
        data = np.array(results["accuracy"])
        mu = np.mean(data)
        sigma = np.std(data)
        worst_case = mu - (3 * sigma)
        
        return {
            "mean": mu,
            "std_dev": sigma,
            "3_sigma_low": worst_case,
            "margin": worst_case - REQ_THRESHOLD,
            "raw_data": results["accuracy"]
        }