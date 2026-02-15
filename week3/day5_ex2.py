import numpy as np

def normalize(v: np.ndarray) -> np.ndarray:
    s = v.sum()
    if s == 0:
        raise ValueError("Cannot normalize: sum is 0 (check model).")
    return v / s

# ----- Model (same as your filtering)
# States: R1, R2, R3  -> indices 0,1,2

T = np.array([
    [0.0, 0.0, 0.0],  # to R1
    [0.5, 0.8, 0.3],  # to R2
    [0.5, 0.2, 0.7],  # to R3
], dtype=float)

M = np.array([
    [0.0, 0.5, 0.5],  # actual R1
    [0.0, 0.9, 0.1],  # actual R2
    [0.0, 0.1, 0.9],  # actual R3
], dtype=float)

# Observations: {R2, R3, R3, R2, R3} -> indices {1,2,2,1,2}
obs = np.array([1, 2, 2, 1, 2], dtype=int)
N = len(obs)

# ----- FORWARD (alpha) -----
# Notes convention: start known at R1, and do "no pred" at k=1
bel0 = np.array([1.0, 0.0, 0.0])

alpha = []  # unnormalized forward messages α_k
bel_filt = []  # normalized filtering beliefs (for sanity)

# k=1: no prediction, just incorporate z1
a1 = bel0 * M[:, obs[0]]
alpha.append(a1)                 # store unnormalized α1
bel = normalize(a1)
bel_filt.append(bel)

# k=2..N: predict then update
for k in range(1, N):
    bel_bar = T @ bel
    ak = bel_bar * M[:, obs[k]]
    alpha.append(ak)
    bel = normalize(ak)
    bel_filt.append(bel)

print("Filtered (from forward pass):")
for k, b in enumerate(bel_filt, start=1):
    print(f"k={k}: {b}")

# ----- BACKWARD (beta) -----
beta = [None] * N
beta[-1] = np.ones(3)  # β_N = [1,1,1]

for k in range(N - 2, -1, -1):
    # β_k(i) = sum_j T[j,i] * M[j, z_{k+1}] * β_{k+1}(j)
    beta_k = np.zeros(3)
    for i in range(3):          # current state i at time k
        s = 0.0
        for j in range(3):      # next state j at time k+1
            s += T[j, i] * M[j, obs[k + 1]] * beta[k + 1][j]
        beta_k[i] = s
    beta[k] = beta_k

print("\nBackward steps (beta):")
for k, b in enumerate(beta, start=1):
    print(f"beta k={k}: {b}")

# ----- SMOOTHED -----
print("\nSmoothed:")
for k in range(N):
    numer = alpha[k] * beta[k]        # elementwise
    sm = normalize(numer)
    print(f"smoothed k={k+1}: {sm}")
