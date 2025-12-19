import pickle
import numpy as np

#NOT CURRENTLY UPDATED OR BEING USED


p = "logs/run_2025-10-28_23-57-30/run_hist.pkl"  # set correct path
with open(p, "rb") as f:
    run_hist = pickle.load(f)

# inspect keys and types
print(run_hist.keys())
pop_hist = run_hist["pop_hist"]
pop_rew_hist = run_hist["pop_rew_hist"]
opt_params = run_hist["opt_params"]

print("pop_hist.shape:", getattr(pop_hist, "shape", None))
print("pop_rew_hist.shape:", getattr(pop_rew_hist, "shape", None))
print("opt_params type:", type(opt_params))

final_rewards = pop_rew_hist[-1]              # shape (pop_size,)
best_idx = int(final_rewards.argmax())
best_reward = float(final_rewards[best_idx])
best_params = pop_hist[-1, best_idx]          # vector of length num_dims

print(f"Best Parameters are {best_params[0]} and {best_params[1]} with a reward of {best_reward}")