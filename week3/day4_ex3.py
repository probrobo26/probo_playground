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
