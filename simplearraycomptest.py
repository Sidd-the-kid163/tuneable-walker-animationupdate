import numpy as np
import yaml
import numpy as np

# Load YAML
with open('All_Gaits/gait_1_1/params_gait_1_1.yaml') as f:
    data = yaml.safe_load(f)

gait = data['domain'][0]
Real = np.reshape(gait['aposition'], (6, 8))
Gen = np.random.rand(6, 8)

# --- Similarity Metrics ---

# 1. Mean Squared Error (lower = more similar)
def mse(a, b):
    return np.mean((a - b) ** 2)

mse_value = mse(Real, Gen)
print(f"Mean Squared Error: {mse_value}")