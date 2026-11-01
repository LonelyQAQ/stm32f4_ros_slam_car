"""把 vqf.c 里用 NaN 当"滤波器未初始化"标记的写法换成显式 bool 标志。

原因：Keil 的 AC6 默认按 fast-math 语义编译，`isnan()` 会被折叠成 false，
而 VQF 靠 NaN 判定首次初始化 —— 一旦折叠就会把 NaN 传进四元数。
改成显式标志后不再依赖任何浮点语义，也不需要工程级编译选项。
"""
import io

p = 'firmware/vqf/vqf.c'
s = io.open(p, encoding='utf-8').read()
n = 0

def rep(old, new, count=1):
    global s, n
    assert s.count(old) == count, 'anchor %r count=%d' % (old[:60], s.count(old))
    s = s.replace(old, new)
    n += count

# 1) state 结构体：加 6 个初始化标志
rep("""    bool restDetected;
    bool magDistDetected;""",
    """    bool restDetected;
    bool magDistDetected;
    /* 各低通滤波器的"是否已经完成初始化"标志（替代上游的 NaN 哨兵） */
    bool accLpInit;
    bool restGyrLpInit;
    bool restAccLpInit;
    bool motionBiasEstRLpInit;
    bool motionBiasEstBiasLpInit;
    bool magNormDipLpInit;""")

# 2) filterVec：加 bool* 参数，用标志代替 isnan
rep("""static void filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts, const vqf_double_t b[3],
                    const vqf_double_t a[2], vqf_double_t state[], vqf_real_t out[])
{
    // assert(N>=2);

    // to avoid depending on a single sample, average the first samples (for duration tau)
    // and then use this average to calculate the filter initial state
    if (isnan(state[0])) { // initialization phase
        if (isnan(state[1])) { // first sample
            state[1] = 0; // state[1] is used to store the sample count
            for(size_t i = 0; i < N; i++) {
                state[2+i] = 0; // state[2+i] is used to store the sum
            }
        }
        state[1]++;
        for (size_t i = 0; i < N; i++) {
            state[2+i] += x[i];
            out[i] = state[2+i]/state[1];
        }
        if (state[1]*Ts >= tau) {
            for(size_t i = 0; i < N; i++) {
               filterInitialState(out[i], b, a, state+2*i);
            }
        }
        return;
    }""",
    """static void filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts, const vqf_double_t b[3],
                    const vqf_double_t a[2], vqf_double_t state[], bool *inited, vqf_real_t out[])
{
    // assert(N>=2);

    // to avoid depending on a single sample, average the first samples (for duration tau)
    // and then use this average to calculate the filter initial state
    if (!*inited) { // initialization phase
        if (state[1] == 0) { // first sample
            for(size_t i = 0; i < N; i++) {
                state[2+i] = 0; // state[2+i] is used to store the sum
            }
        }
        state[1]++; // state[1] is used to store the sample count
        for (size_t i = 0; i < N; i++) {
            state[2+i] += x[i];
            out[i] = state[2+i]/state[1];
        }
        if (state[1]*Ts >= tau) {
            for(size_t i = 0; i < N; i++) {
               filterInitialState(out[i], b, a, state+2*i);
            }
            /* 用完之后 state[0]/state[1] 已被真正的滤波状态覆盖 */
            *inited = true;
        }
        return;
    }""")

# 3) 六个调用点
rep("""        filterVec(gyr, 3, params.restFilterTau, coeffs.gyrTs, coeffs.restGyrLpB, coeffs.restGyrLpA,
                  state.restGyrLpState, state.restLastGyrLp);""",
    """        filterVec(gyr, 3, params.restFilterTau, coeffs.gyrTs, coeffs.restGyrLpB, coeffs.restGyrLpA,
                  state.restGyrLpState, &state.restGyrLpInit, state.restLastGyrLp);""")

rep("""        filterVec(acc, 3, params.restFilterTau, coeffs.accTs, coeffs.restAccLpB, coeffs.restAccLpA,
                  state.restAccLpState, state.restLastAccLp);""",
    """        filterVec(acc, 3, params.restFilterTau, coeffs.accTs, coeffs.restAccLpB, coeffs.restAccLpA,
                  state.restAccLpState, &state.restAccLpInit, state.restLastAccLp);""")

rep("""    filterVec(accEarth, 3, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.accLpState, state.lastAccLp);""",
    """    filterVec(accEarth, 3, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.accLpState,
              &state.accLpInit, state.lastAccLp);""")

rep("""        filterVec(R, 9, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.motionBiasEstRLpState, R);
        filterVec(biasLp, 2, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.motionBiasEstBiasLpState,
                  biasLp);""",
    """        filterVec(R, 9, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.motionBiasEstRLpState,
                  &state.motionBiasEstRLpInit, R);
        filterVec(biasLp, 2, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.motionBiasEstBiasLpState,
                  &state.motionBiasEstBiasLpInit, biasLp);""")

rep("""            filterVec(state.magNormDip, 2, params.magCurrentTau, coeffs.magTs, coeffs.magNormDipLpB,
                      coeffs.magNormDipLpA, state.magNormDipLpState, state.magNormDip);""",
    """            filterVec(state.magNormDip, 2, params.magCurrentTau, coeffs.magTs, coeffs.magNormDipLpB,
                      coeffs.magNormDipLpA, state.magNormDipLpState, &state.magNormDipLpInit, state.magNormDip);""")

# 4) NaN 填充 -> 0 填充（标志位由 resetState / setter 置 false）
for a, b in [
    ("vqf_fill_double(state.motionBiasEstRLpState, 9*2, NaN);", "vqf_fill_double(state.motionBiasEstRLpState, 9*2, 0.0);"),
    ("vqf_fill_double(state.motionBiasEstBiasLpState, 2*2, NaN);", "vqf_fill_double(state.motionBiasEstBiasLpState, 2*2, 0.0);"),
    ("vqf_fill_double(state.restGyrLpState, 3*2, NaN);", "vqf_fill_double(state.restGyrLpState, 3*2, 0.0);"),
    ("vqf_fill_double(state.restAccLpState, 3*2, NaN);", "vqf_fill_double(state.restAccLpState, 3*2, 0.0);"),
    ("vqf_fill_double(state.magNormDipLpState, 2*2, NaN);", "vqf_fill_double(state.magNormDipLpState, 2*2, 0.0);"),
    ("vqf_fill_double(state.accLpState, 3*2, NaN);", "vqf_fill_double(state.accLpState, 3*2, 0.0);"),
    ("vqf_fill_real(state.restLastGyrLp, 3, NaN);", "vqf_fill_real(state.restLastGyrLp, 3, 0.0);"),
    ("vqf_fill_double(coeffs.magNormDipLpB, 3, NaN);", "vqf_fill_double(coeffs.magNormDipLpB, 3, 0.0);"),
    ("vqf_fill_double(coeffs.magNormDipLpA, 2, NaN);", "vqf_fill_double(coeffs.magNormDipLpA, 2, 0.0);"),
]:
    rep(a, b, s.count(a))

# 5) resetState()：清标志
rep("""void resetState()
{
    quatSetToIdentity(state.gyrQuat);""",
    """void resetState()
{
    state.accLpInit = false;
    state.restGyrLpInit = false;
    state.restAccLpInit = false;
    state.motionBiasEstRLpInit = false;
    state.motionBiasEstBiasLpInit = false;
    state.magNormDipLpInit = false;

    quatSetToIdentity(state.gyrQuat);""")

io.open(p, 'w', encoding='utf-8', newline='').write(s)
print('replacements:', n)
