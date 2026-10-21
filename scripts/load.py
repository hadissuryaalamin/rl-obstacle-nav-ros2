#!/usr/bin/env python3
"""
Model Loading and Inference Script for AutoNav Tutor

This script loads a trained PPO model and performs inference on the TutorBotEnv.
It supports:
  - Loading trained models from disk
  - Running benchmark tests with detailed logging
  - Computing success metrics (win rate, crash detection)
  - Deploying the model for real-time control
  
Usage:
  python3 load.py --model path/to/model.zip --stats path/to/stats.pkl [--tests 50]
  python3 load.py  # Uses default paths from configuration
"""

import os
import sys
import csv
import time
import argparse
import datetime
import rclpy
from stable_baselines3 import PPO, A2C, DQN
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from rl_env import TutorBotEnv

# --- DEFAULT CONFIGURATION ---
DEFAULT_MODEL_PATH = "models/PPO/final_model.zip"
DEFAULT_STATS_PATH = "models/PPO/vec_normalize.pkl"
DEFAULT_OUTPUT_DIR = "benchmark_logs"
DEFAULT_TOTAL_TESTS = 50
DEFAULT_MAX_STEPS = 500

class ModelLoader:
    """Handles loading and managing trained RL models."""
    
    def __init__(self, model_path, stats_path=None, model_type="PPO"):
        """
        Initialize model loader.
        
        Args:
            model_path (str): Path to saved model (.zip file)
            stats_path (str): Path to VecNormalize statistics (.pkl file)
            model_type (str): Type of model (PPO, A2C, DQN)
        """
        self.model_path = model_path
        self.stats_path = stats_path
        self.model_type = model_type
        self.model = None
        self.env = None
        
    def load(self):
        """Load the trained model and environment."""
        print(f"[INFO] Loading {self.model_type} model from: {self.model_path}")
        
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Model not found: {self.model_path}")
        
        # Create environment
        print("[INFO] Initializing TutorBotEnv...")
        self.env = TutorBotEnv()
        self.env = DummyVecEnv([lambda: self.env])
        
        # Load normalization statistics if available
        if self.stats_path and os.path.exists(self.stats_path):
            print(f"[INFO] Loading normalization stats from: {self.stats_path}")
            self.env = VecNormalize.load(self.stats_path, self.env)
            self.env.training = False
            self.env.norm_reward = False
        else:
            print("[WARN] No normalization stats found. Running without normalization.")
        
        # Load model based on type
        if self.model_type == "PPO":
            self.model = PPO.load(self.model_path, env=self.env)
        elif self.model_type == "A2C":
            self.model = A2C.load(self.model_path, env=self.env)
        elif self.model_type == "DQN":
            self.model = DQN.load(self.model_path, env=self.env)
        else:
            raise ValueError(f"Unknown model type: {self.model_type}")
        
        print("[SUCCESS] Model loaded successfully!")
        return self.model, self.env
    
    def predict(self, obs, deterministic=True):
        """
        Get prediction from the model.
        
        Args:
            obs: Current observation
            deterministic (bool): Whether to use deterministic policy
            
        Returns:
            tuple: (action, _value)
        """
        return self.model.predict(obs, deterministic=deterministic)

def run_benchmark(model_loader, total_tests=50, max_steps=500, output_dir=None):
    """
    Run benchmark tests on the loaded model.
    
    Args:
        model_loader (ModelLoader): Loaded model instance
        total_tests (int): Number of test episodes
        max_steps (int): Maximum steps per episode
        output_dir (str): Directory to save results
        
    Returns:
        dict: Benchmark results (wins, crashes, accuracy, etc.)
    """
    model, env = model_loader.model, model_loader.env
    
    # Setup output directory
    if output_dir is None:
        output_dir = DEFAULT_OUTPUT_DIR
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    csv_filename = f"{output_dir}/benchmark_{timestamp}.csv"
    
    print(f"\n{'='*50}")
    print(f"STARTING BENCHMARK ({total_tests} Episodes)")
    print(f"{'='*50}")
    print(f"Model Type: {model_loader.model_type}")
    print(f"Max Steps per Episode: {max_steps}")
    print(f"Results will be saved to: {csv_filename}\n")

    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        # HEADER ROW
        writer.writerow(["Episode", "Outcome", "Final_Reward", "Steps"])

        wins = 0
        crashes = 0
        timeouts = 0
        total_steps_taken = 0

        # 3. Testing Loop
        for i in range(1, total_tests + 1):
            obs = env.reset()
            done = False
            steps_taken = 0
            episode_reward = 0.0
            
            while not done and steps_taken < max_steps:
                action, _ = model_loader.predict(obs, deterministic=True)
                obs, rewards, dones, info = env.step(action)
                episode_reward += rewards[0]
                steps_taken += 1
                
                if dones[0]:
                    done = True
                
            total_steps_taken += steps_taken
            
            # Determine Outcome
            outcome = ""
            if episode_reward >= 90:
                wins += 1
                outcome = "WIN"
                print(f"  [Episode {i:3d}] WIN     | Reward: {episode_reward:7.2f} | Steps: {steps_taken}")
            elif episode_reward <= -90:
                crashes += 1
                outcome = "CRASH"
                print(f"  [Episode {i:3d}] CRASH   | Reward: {episode_reward:7.2f} | Steps: {steps_taken}")
            else:
                timeouts += 1
                outcome = "TIMEOUT"
                print(f"  [Episode {i:3d}] TIMEOUT | Reward: {episode_reward:7.2f} | Steps: {steps_taken}")
            
            # SAVE TO CSV
            writer.writerow([i, outcome, f"{episode_reward:.2f}", steps_taken])

    # 4. Final Report
    accuracy = (wins / total_tests) * 100
    avg_steps = total_steps_taken / total_tests if total_tests > 0 else 0
    
    results = {
        'total_tests': total_tests,
        'wins': wins,
        'crashes': crashes,
        'timeouts': timeouts,
        'accuracy': accuracy,
        'avg_steps': avg_steps,
        'csv_file': csv_filename
    }
    
    print(f"\n{'='*50}")
    print(f"BENCHMARK RESULTS")
    print(f"{'='*50}")
    print(f"Total Tests:    {total_tests}")
    print(f"Wins:           {wins}")
    print(f"Crashes:        {crashes}")
    print(f"Timeouts:       {timeouts}")
    print(f"Win Rate:       {accuracy:.1f}%")
    print(f"Avg Steps:      {avg_steps:.1f}")
    print(f"Results saved:  {csv_filename}")
    print(f"{'='*50}\n")
    
    return results

def deploy_model(model_loader, max_episodes=10, max_steps=500):
    """
    Deploy the model for real-time inference.
    
    Args:
        model_loader (ModelLoader): Loaded model instance
        max_episodes (int): Number of episodes to run
        max_steps (int): Maximum steps per episode
    """
    model, env = model_loader.model, model_loader.env
    
    print(f"\n{'='*50}")
    print(f"DEPLOYING MODEL (Real-time Inference)")
    print(f"{'='*50}\n")
    
    for episode in range(1, max_episodes + 1):
        obs = env.reset()
        done = False
        steps = 0
        episode_reward = 0.0
        
        print(f"Episode {episode}/{max_episodes}")
        
        while not done and steps < max_steps:
            action, _ = model_loader.predict(obs, deterministic=True)
            obs, rewards, dones, info = env.step(action)
            episode_reward += rewards[0]
            steps += 1
            
            if dones[0]:
                done = True
        
        print(f"  Steps: {steps} | Reward: {episode_reward:.2f}")

def main():
    parser = argparse.ArgumentParser(
        description="Load and run inference with trained AutoNav Tutor models"
    )
    parser.add_argument(
        '--model', 
        type=str, 
        default=DEFAULT_MODEL_PATH,
        help=f'Path to trained model (.zip file). Default: {DEFAULT_MODEL_PATH}'
    )
    parser.add_argument(
        '--stats',
        type=str,
        default=DEFAULT_STATS_PATH,
        help=f'Path to VecNormalize stats (.pkl file). Default: {DEFAULT_STATS_PATH}'
    )
    parser.add_argument(
        '--model-type',
        type=str,
        choices=['PPO', 'A2C', 'DQN'],
        default='PPO',
        help='Type of model to load. Default: PPO'
    )
    parser.add_argument(
        '--tests',
        type=int,
        default=DEFAULT_TOTAL_TESTS,
        help=f'Number of benchmark tests. Default: {DEFAULT_TOTAL_TESTS}'
    )
    parser.add_argument(
        '--max-steps',
        type=int,
        default=DEFAULT_MAX_STEPS,
        help=f'Maximum steps per episode. Default: {DEFAULT_MAX_STEPS}'
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default=DEFAULT_OUTPUT_DIR,
        help=f'Output directory for results. Default: {DEFAULT_OUTPUT_DIR}'
    )
    parser.add_argument(
        '--deploy',
        action='store_true',
        help='Run in deployment mode (continuous inference)'
    )
    parser.add_argument(
        '--episodes',
        type=int,
        default=10,
        help='Number of deployment episodes. Default: 10'
    )
    
    args = parser.parse_args()
    
    try:
        rclpy.init()
        
        # Load model
        loader = ModelLoader(args.model, args.stats, args.model_type)
        loader.load()
        
        # Run benchmark or deployment
        if args.deploy:
            deploy_model(loader, max_episodes=args.episodes, max_steps=args.max_steps)
        else:
            run_benchmark(
                loader,
                total_tests=args.tests,
                max_steps=args.max_steps,
                output_dir=args.output_dir
            )
    
    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()