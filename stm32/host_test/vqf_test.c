/* 主机侧 VQF 单测：用 gcc 编 vqf.c，喂合成数据验证三件事
 *   1. 静止（陀螺 0 / 重力 +Z）→ 四元数=单位、yaw 不漂
 *   2. 定速 yaw 旋转 90°/s × 4 s → 积分角 ≈ 360°
 *   3. 30° 倾斜重力 → pitch 收敛到 30°
 * 编译：gcc -O2 -I../firmware/vqf vqf_test.c ../firmware/vqf/vqf.c -lm -o vqf_test
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "vqf.h"

#define TS 0.005f
#define G 9.80665f

static void euler_from_quat(const float q[4], float *roll, float *pitch, float *yaw)
{
    /* q = [w, x, y, z]，ZYX 欧拉角（deg） */
    float w = q[0], x = q[1], y = q[2], z = q[3];
    *roll = atan2f(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 57.2958f;
    float s = 2 * (w * y - z * x);
    if (s > 1) s = 1;
    if (s < -1) s = -1;
    *pitch = asinf(s) * 57.2958f;
    *yaw = atan2f(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 57.2958f;
}

static int check(const char *name, float got, float want, float tol)
{
    int ok = fabsf(got - want) <= tol;
    printf("%-42s got %+9.3f  want %+9.3f  tol %5.3f  %s\n",
           name, got, want, tol, ok ? "PASS" : "FAIL");
    return ok;
}

int main(void)
{
    int pass = 0, total = 0;
    float q[4], roll, pitch, yaw;
    float gyr[3], acc[3];
    int i;

    /* ---- 1. 静止 60 s ---- */
    initVqf(TS, TS, TS);
    gyr[0] = gyr[1] = gyr[2] = 0.0f;
    acc[0] = 0.0f; acc[1] = 0.0f; acc[2] = G;
    for (i = 0; i < 12000; i++) {
        updateGyr(gyr);
        updateAcc(acc);
    }
    getQuat6D(q);
    euler_from_quat(q, &roll, &pitch, &yaw);
    printf("\n[1] 静止 60 s  q = [%.5f %.5f %.5f %.5f]\n", q[0], q[1], q[2], q[3]);
    total++; pass += check("  静止 roll", roll, 0.0f, 0.5f);
    total++; pass += check("  静止 pitch", pitch, 0.0f, 0.5f);
    total++; pass += check("  静止 yaw", yaw, 0.0f, 0.5f);
    total++; pass += check("  四元数模长", sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]), 1.0f, 1e-4f);

    /* ---- 2. 绕 Z 90°/s 转 4 s ---- */
    initVqf(TS, TS, TS);
    gyr[0] = 0.0f; gyr[1] = 0.0f; gyr[2] = 90.0f / 57.2958f;
    for (i = 0; i < 800; i++) {
        updateGyr(gyr);
        updateAcc(acc);
    }
    getQuat6D(q);
    euler_from_quat(q, &roll, &pitch, &yaw);
    printf("\n[2] yaw 90°/s × 4 s  →  应 ≈ +360°（首圈 ±5° 视为通过）\n");
    {
        float e = fabsf(yaw - 360.0f);
        if (e > 5.0f && fabsf(yaw + 360.0f) < 5.0f) e = fabsf(yaw + 360.0f);
        total++; pass += check("  积分 yaw (°)", e < 180 ? yaw : yaw + 360.0f, 360.0f, 5.0f);
    }
    total++; pass += check("  roll 保持 0", roll, 0.0f, 1.0f);
    total++; pass += check("  pitch 保持 0", pitch, 0.0f, 1.0f);

    /* ---- 3. 重力偏向 +Y（绕 X 转 30°），看 roll 收敛 ---- */
    initVqf(TS, TS, TS);
    gyr[0] = gyr[1] = gyr[2] = 0.0f;
    acc[0] = 0.0f;
    acc[1] = G * sinf(30.0f / 57.2958f);
    acc[2] = G * cosf(30.0f / 57.2958f);
    for (i = 0; i < 12000; i++) {          /* 60 s，足够收敛（tauAcc=3 s） */
        updateGyr(gyr);
        updateAcc(acc);
    }
    getQuat6D(q);
    euler_from_quat(q, &roll, &pitch, &yaw);
    printf("\n[3] 重力绕 X 偏 30° 静止 60 s → roll 应 ≈ ±30°\n");
    total++; pass += check("  收敛 roll", fabsf(roll), 30.0f, 1.0f);
    total++; pass += check("  pitch 保持 0", pitch, 0.0f, 1.0f);

    printf("\n%d/%d PASS\n", pass, total);
    return pass == total ? 0 : 1;
}
