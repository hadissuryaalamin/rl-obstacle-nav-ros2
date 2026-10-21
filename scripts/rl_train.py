#!/usr/bin/env python3
import os
import time
import rclpy
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.monitor import Monitor
from rl_env import TutorBotEnv

# --- PATH DEFINITIONS ---
LOG_DIR = "training_logs"
MODEL_DIR = "models/PPO"

class DetailedLogCallback(BaseCallback):
    def __init__(self, save_freq, save_path, verbose=0):
        super(DetailedLogCallback, self).__init__(verbose)
        self.episode_rewards = []
        self.current_episode_reward = 0
        self.episode_counts = 0
        self.save_freq = save_freq
        self.save_path = save_path
        
        # FPS Tracking
        self.last_time = time.time()
        self.last_step_count = 0

    def _on_step(self) -> bool:
        # 1. Accumulate Reward
        reward = self.locals['rewards'][0]
        self.current_episode_reward += reward

        # 2. Episode Finished Logic
        if self.locals['dones'][0]:
            self.episode_counts += 1
            self.episode_rewards.append(self.current_episode_reward)
            avg_reward = np.mean(self.episode_rewards[-5:])
            
            # --- CALCULATE FPS ---
            current_time = time.time()
            dt = current_time - self.last_time
            d_steps = self.n_calls - self.last_step_count
            
            if dt > 0:
                fps = d_steps / dt
            else:
                fps = 0.0
            
            # Reset counters for next episode
            self.last_time = current_time
            self.last_step_count = self.n_calls
            # ---------------------
            
            print(f"  --> EPISODE {self.episode_counts} FINISHED. Reward: {self.current_episode_reward:.2f} | Avg: {avg_reward:.2f} | FPS: {fps:.0f}")
            self.current_episode_reward = 0

        # 3. AUTO-SAVE 
        if self.n_calls % self.save_freq == 0:
            model_name = f"tutor_model_{self.n_calls}_steps"
            print(f"\n[CHECKPOINT] Saving model and stats to {model_name}...")
            self.model.save(os.path.join(self.save_path, model_name))
            env = self.model.get_env()
            env.save(os.path.join(self.save_path, "vec_normalize.pkl"))
            print("[CHECKPOINT] Done.\n")

        return True

def main():
    rclpy.init()
    
    if not os.path.exists(LOG_DIR): os.makedirs(LOG_DIR)
    if not os.path.exists(MODEL_DIR): os.makedirs(MODEL_DIR)

    # Setup Environment
    env = TutorBotEnv()
    env = Monitor(env, LOG_DIR)
    env = DummyVecEnv([lambda: env])
    env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)

    callback = DetailedLogCallback(save_freq=20000, save_path=MODEL_DIR)
    model = PPO("MlpPolicy", env, verbose=0) 
    
    print("-------------------------------------------------")
    print(f"STARTING TRAINING...")
    print(f" - Logs: {LOG_DIR}/monitor.csv")
    print(f" - Models: {MODEL_DIR}")
    print("-------------------------------------------------")
    
    try:
        TIMESTEPS = 1000000
        model.learn(total_timesteps=TIMESTEPS, callback=callback)
        model.save(f"{MODEL_DIR}/final_model")
        env.save(f"{MODEL_DIR}/vec_normalize.pkl")
        print("Training Complete.")
        
    except KeyboardInterrupt:
        print("\n[!] Interrupted. Saving...")
        model.save(f"{MODEL_DIR}/interrupted_model")
        env.save(f"{MODEL_DIR}/vec_normalize.pkl")
        print("Saved.")

if __name__ == '__main__':
    main()