import numpy as np

def my_cov(m, rowvar=True, bias=False):
    """
    A simple covariance implementation similar to np.cov for common cases.
    - m: array-like (1D or 2D)
    - rowvar: if True, each row represents a variable; columns are observations
    - bias: if False -> normalize by (N-1); if True -> normalize by N
    """
    X = np.asarray(m, dtype=float)

    # If 1D, treat as single variable
    if X.ndim == 1:
        X = X.reshape(1, -1)  # 1 x N

    # If variables are in columns instead, transpose
    if not rowvar:
        X = X.T  # now vars are rows

    n_obs = X.shape[1]
    if n_obs < 2 and not bias:
        raise ValueError("Need at least 2 observations for unbiased covariance (N-1).")

    # Center each variable
    mean = X.mean(axis=1, keepdims=True)
    Xc = X - mean

    denom = n_obs if bias else (n_obs - 1)
    return (Xc @ Xc.T) / denom

A = np.array([[0, 2], [1, 1], [2, 0]]).T

print("my_cov:\n", my_cov(A))
print("np.cov:\n", np.cov(A))
print("close?", np.allclose(my_cov(A), np.cov(A)))


def my_corrcoef(m, rowvar=True, bias=False):
    C = my_cov(m, rowvar=rowvar, bias=bias)
    std = np.sqrt(np.diag(C))
    denom = np.outer(std, std)

    # Avoid divide-by-zero if any variable has zero variance
    with np.errstate(divide='ignore', invalid='ignore'):
        R = C / denom
        R[denom == 0] = 0.0
    # Force diagonal to 1 when variance isn't zero
    for i in range(R.shape[0]):
        if std[i] != 0:
            R[i, i] = 1.0
    return R


# test
A = np.array([[0, 2, 1, 3],
              [1, 1, 2, 1]])

print("my_corrcoef:\n", my_corrcoef(A))
print("np.corrcoef:\n", np.corrcoef(A))
print("close?", np.allclose(my_corrcoef(A), np.corrcoef(A)))


'''
my_cov:
 [[ 1. -1.]
 [-1.  1.]]
np.cov:
 [[ 1. -1.]
 [-1.  1.]]
close? True
my_corrcoef:
 [[ 1.         -0.25819889]
 [-0.25819889  1.        ]]
np.corrcoef:
 [[ 1.         -0.25819889]
 [-0.25819889  1.        ]]
close? True

'''