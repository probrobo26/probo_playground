import numpy as np

bel = np.array([0.985, 0.015], dtype=float)   # prior
likelihood = np.array([0.25, 1.0], dtype=float)  # matches the example behavior

print("N | P(normal) | P(faulty)")
print("---------------------------")

# N=1 shown as the prior (no update)
print(f"{1:2d} | {bel[0]:.6f} | {bel[1]:.6f}")

# Apply updates for N=2..10
for k in range(2, 11):
    bel = bel * likelihood
    bel = bel / bel.sum()
    print(f"{k:2d} | {bel[0]:.6f} | {bel[1]:.6f}")
