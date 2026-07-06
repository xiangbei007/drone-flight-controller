#include "zf_common_headfile.h"

#pragma location = 0x28001000
cam_share_t cam_share;

#pragma location = 0x28001020
tof_share_t tof_share;   // TOF 高度回传（7-0 写 / 7-1 读），独立 cache line

#define MIN(a, b)        (((a) < (b)) ? (a) : (b))
#define MAX(a, b)        (((a) > (b)) ? (a) : (b))

#define BOUNDARY_NUM     ((MT9V03X_W + MT9V03X_H) * 2)
#define BINARY_THRESHOLD 58.7//100
//#define LED1             P19_0
#define LINE_LENGTH      38
#define MAX_POINTS       4000        // 单个连通域最大像素数
#define INNER_OFFSET     4           // 方框内边界再向内 INNER_OFFSET 像素

#define PI_F             3.14159265f

uint8 image_copy[MT9V03X_H][MT9V03X_W];

typedef struct
{
    int16 corner_x[4], corner_y[4];  // 方框 4 角点(图像坐标，按顺序)
    int16 cx, cy;                    // V 形质心
    float angle;                     // V 输出方向(主轴 +90°)
    uint8 valid;
    uint16 score;
} v_object_t;

uint8 xy1[BOUNDARY_NUM], xy2[BOUNDARY_NUM], xy3[BOUNDARY_NUM];
uint8 yx1[BOUNDARY_NUM], yx2[BOUNDARY_NUM], yx3[BOUNDARY_NUM];

uint8 img_bin[MT9V03X_IMAGE_SIZE];
static uint8 vis[MT9V03X_H][MT9V03X_W];
v_object_t target_v;

// 连通域点集（同时作为 BFS 队列）
static int16 pts_x[MAX_POINTS];
static int16 pts_y[MAX_POINTS];

// 方框线坐标备份（用于 PCA 后精确还原 img，避免 pts_x/pts_y 被 PCA 复用覆盖）
static int16 frame_px[MAX_POINTS];
static int16 frame_py[MAX_POINTS];

// ─────────────────────────────────────────
// 二值化
// ─────────────────────────────────────────
static void img_binarize(uint8 *src, uint8 *dst)
{
    uint16 i;
    for(i = 0; i < MT9V03X_IMAGE_SIZE; i++)
        dst[i] = (src[i] <= BINARY_THRESHOLD) ? 0 : 255;
}

// ─────────────────────────────────────────
// 求点集的最小外接旋转矩形（角度遍历法，正方形适用）
// 返回：角度 theta、中心 cx/cy、半宽 hw、半高 hh
//
// ★ 两阶段搜索：先 2° 粗搜索找到大致最优角度，再在 ±2° 内 0.2° 细搜索
//   量化误差从 ±1°（粗搜索）降到 ±0.1°（细搜索），完全消除"非 0/90° 时偏移大"的问题
// ─────────────────────────────────────────
static void min_area_rect(int16 *px, int16 *py, uint16 cnt,
                          float *theta_out, float *cx_out, float *cy_out,
                          float *hw_out, float *hh_out)
{
    float best_area = 1e18f;
    float best_theta = 0;
    float best_umin = 0, best_umax = 0, best_vmin = 0, best_vmax = 0;

    // ── 第 1 阶段：在 [0°, 90°) 内按 2° 步进粗搜索 ──
    for(int16 deg = 0; deg < 90; deg += 2)
    {
        float t = deg * PI_F / 180.0f;
        float ct = cosf(t), st = sinf(t);

        float u_min =  1e9f, u_max = -1e9f;
        float v_min =  1e9f, v_max = -1e9f;
        for(uint16 i = 0; i < cnt; i++)
        {
            float u =  px[i] * ct + py[i] * st;
            float v = -px[i] * st + py[i] * ct;
            if(u < u_min) u_min = u;
            if(u > u_max) u_max = u;
            if(v < v_min) v_min = v;
            if(v > v_max) v_max = v;
        }
        float area = (u_max - u_min) * (v_max - v_min);
        if(area < best_area)
        {
            best_area  = area;
            best_theta = t;
            best_umin  = u_min;
            best_umax  = u_max;
            best_vmin  = v_min;
            best_vmax  = v_max;
        }
    }

    // ── 第 2 阶段：在粗搜索最优角度 ± 2° 内按 0.2° 细搜索 ──
    // 步进 0.2° → 量化误差 ±0.1°，对 hw=40 的方框，PCA 区域偏移仅 0.07 像素
    float center_t = best_theta;
    for(int16 i = -10; i <= 10; i++)
    {
        float t = center_t + (i * 0.2f) * PI_F / 180.0f;
        float ct = cosf(t), st = sinf(t);

        float u_min =  1e9f, u_max = -1e9f;
        float v_min =  1e9f, v_max = -1e9f;
        for(uint16 j = 0; j < cnt; j++)
        {
            float u =  px[j] * ct + py[j] * st;
            float v = -px[j] * st + py[j] * ct;
            if(u < u_min) u_min = u;
            if(u > u_max) u_max = u;
            if(v < v_min) v_min = v;
            if(v > v_max) v_max = v;
        }
        float area = (u_max - u_min) * (v_max - v_min);
        if(area < best_area)
        {
            best_area  = area;
            best_theta = t;
            best_umin  = u_min;
            best_umax  = u_max;
            best_vmin  = v_min;
            best_vmax  = v_max;
        }
    }

    // 由旋转坐标系下的中心反推图像坐标系的中心
    float u_c = (best_umin + best_umax) * 0.5f;
    float v_c = (best_vmin + best_vmax) * 0.5f;
    float ct  = cosf(best_theta), st = sinf(best_theta);
    *cx_out = u_c * ct - v_c * st;     // 注意：图像坐标 = R(+theta) * (u,v)
    *cy_out = u_c * st + v_c * ct;
    *hw_out = (best_umax - best_umin) * 0.5f;
    *hh_out = (best_vmax - best_vmin) * 0.5f;
    *theta_out = best_theta;
}

// ─────────────────────────────────────────
// 判定连通域是否为「空心方形方框」，并返回 4 角点 + 旋转参数
// ─────────────────────────────────────────
static uint8 detect_square_frame(int16 *px, int16 *py, uint16 cnt,
                                  int16 corner_x[4], int16 corner_y[4],
                                  float *frame_theta,
                                  float *frame_cx, float *frame_cy,
                                  float *frame_hw, float *frame_hh)
{
    if(cnt < 80) return 0;

    float theta, cx, cy, hw, hh;
    min_area_rect(px, py, cnt, &theta, &cx, &cy, &hw, &hh);

    float w = hw * 2.0f;
    float h = hh * 2.0f;
    if(w < 20.0f || h < 20.0f)         return 0;
    if(w > MT9V03X_W * 0.95f)          return 0;
    if(h > MT9V03X_H * 0.95f)          return 0;

    // 宽高比 ≈ 1（方形）
    float ratio = (w > h) ? (w / h) : (h / w);
    if(ratio > 1.30f) return 0;

    // 空心判定：中心区域黑像素比例应很小
    // 在旋转坐标系内 (u,v)：中心 u=u_c, v=v_c；半宽/半高 = 0.6 * hw/hh 视为内部
    float ct = cosf(theta), st = sinf(theta);
    float u_c =  cx * ct + cy * st;
    float v_c = -cx * st + cy * ct;

    uint16 inner_cnt = 0;
    float inner_hw = hw * 0.6f;
    float inner_hh = hh * 0.6f;
    for(uint16 i = 0; i < cnt; i++)
    {
        float u =  px[i] * ct + py[i] * st;
        float v = -px[i] * st + py[i] * ct;
        float du = u - u_c; if(du < 0) du = -du;
        float dv = v - v_c; if(dv < 0) dv = -dv;
        if(du < inner_hw && dv < inner_hh) inner_cnt++;
    }
    if(inner_cnt > cnt * 0.20f) return 0;          // 中心黑像素太多 → 非空心

    // 面积比：方框像素 ≈ 周长 × 线宽，应远小于 w*h
    float area = w * h;
    float perimeter = 2.0f * (w + h);
    if((float)cnt > area * 0.55f) return 0;        // 太满 → 实心
    if((float)cnt < perimeter * 0.5f) return 0;    // 太稀 → 噪声

    // 4 角点（旋转坐标系 → 图像坐标系：图像坐标 = R(+theta) * (u,v) ）
    float u_min = u_c - hw, u_max = u_c + hw;
    float v_min = v_c - hh, v_max = v_c + hh;
    float corners_u[4] = {u_min, u_max, u_max, u_min};
    float corners_v[4] = {v_min, v_min, v_max, v_max};
    for(int i = 0; i < 4; i++)
    {
        float xx = corners_u[i] * ct - corners_v[i] * st;
        float yy = corners_u[i] * st + corners_v[i] * ct;
        corner_x[i] = (int16)xx;
        corner_y[i] = (int16)yy;
    }

    *frame_theta = theta;
    *frame_cx = cx; *frame_cy = cy;
    *frame_hw = hw; *frame_hh = hh;
    return 1;
}

// ─────────────────────────────────────────
// 估计方框线宽（用方框像素总数 / 周长）
// ─────────────────────────────────────────
static float estimate_line_width(uint16 frame_pixel_cnt, float hw, float hh)
{
    float perimeter = 4.0f * (hw + hh);    // 2*(w+h) = 2*(2hw+2hh)
    if(perimeter < 1.0f) return 1.0f;
    return (float)frame_pixel_cnt / perimeter;
}

// ─────────────────────────────────────────
// 在方框「内边界再向内 INNER_OFFSET 像素」的区域内提取 V 像素，做 PCA
// 输入：img 二值图；frame 旋转参数；
// 输出：V 质心 cx/cy、V 输出方向 angle (主轴+90°)
//
// ★★ 这里有两层过滤，不要混淆：
//   【第 1 层】扫描范围 sx0/sx1/sy0/sy1（for 循环边界）
//             —— 决定"遍历哪些像素"，仅是性能优化，不决定 PCA 区域
//   【第 2 层】旋转坐标系下 du < in_hw && dv < in_hh
//             —— 真正决定"哪些像素纳入 PCA"，严格限制在方框内边界向内 INNER_OFFSET 像素
//
//   外加方框线像素在调用前已被屏蔽为 128，if(img[...]!=0) continue 会直接跳过，
//   形成三重保险，方框线绝不会污染 PCA。
// ─────────────────────────────────────────
static uint8 extract_v_pca(uint8 *img,
                            float f_theta, float f_cx, float f_cy,
                            float f_hw, float f_hh, float line_w,
                            int16 *vx_out, int16 *vy_out, float *vang_out)
{
    float ct = cosf(f_theta), st = sinf(f_theta);
    // 方框中心在旋转坐标系
    float u_c =  f_cx * ct + f_cy * st;
    float v_c = -f_cx * st + f_cy * ct;

    // ── 第 2 层过滤参数：方框内边界向内 INNER_OFFSET 像素的旋转矩形 ──
    // 在旋转坐标系下 |u-u_c| < in_hw && |v-v_c| < in_hh 的像素才纳入 PCA
    float in_hw = f_hw - line_w - INNER_OFFSET;
    float in_hh = f_hh - line_w - INNER_OFFSET;
    if(in_hw < 4.0f || in_hh < 4.0f) return 0;

    int32 sum_x = 0, sum_y = 0;
    uint16 cnt = 0;

    // ── 第 1 层过滤参数：图像坐标下的 AABB 扫描范围（仅决定 for 循环边界）──
    // 关键：方框旋转后，其在图像坐标下的 AABB 半宽 = max(hw,hh) * (|cos|+|sin|)，
    // 最坏情况（45°）= max(hw,hh) * sqrt(2) ≈ 1.414。所以必须用 1.42 倍包络。
    //
    // 若用原来的 f_hw + line_w + 2（即 1.0 倍），方框旋转 45° 时内部矩形对角的
    // V 像素会被 for 循环漏掉，导致 PCA 输入像素分布不对称 → V 中心点偏移。
    //
    // 注意：扫描范围扩大到 1.42 倍并不会"扫到方框边线"，因为：
    //   1) 方框线像素已被设为 128，if(img!=0) continue 直接跳过；
    //   2) 旋转坐标系下的第 2 层过滤 du<in_hw && dv<in_hh 仍严格生效；
    //   范围扩大只是确保"内部矩形的对角 V 像素不会被 for 循环漏访问"。
    float max_h = (f_hw > f_hh) ? f_hw : f_hh;
    float scan_r = max_h * 1.42f + line_w + 2.0f;
    int16 sx0 = (int16)(f_cx - scan_r);
    int16 sx1 = (int16)(f_cx + scan_r);
    int16 sy0 = (int16)(f_cy - scan_r);
    int16 sy1 = (int16)(f_cy + scan_r);
    if(sx0 < 0) sx0 = 0; if(sx1 >= MT9V03X_W) sx1 = MT9V03X_W - 1;
    if(sy0 < 0) sy0 = 0; if(sy1 >= MT9V03X_H) sy1 = MT9V03X_H - 1;

    // 复用 pts_x/pts_y 存 V 像素（覆盖原 BFS 数据，已经用完了）
    // 第 3 层过滤：只用方框内部 1.05*in_hw / 1.05*in_hh 范围内的黑像素，留 5% 余量
    //   配合 INNER_OFFSET=4 + 方框线屏蔽为 128，确保不会扫到方框线外的黑点
    for(int16 y = sy0; y <= sy1; y++)
    {
        for(int16 x = sx0; x <= sx1; x++)
        {
            // 第 0 重过滤：跳过非黑像素（含被屏蔽为 128 的方框线像素）
            if(img[y * MT9V03X_W + x] != 0) continue;
            // 旋转到方框主轴坐标系
            float u =  x * ct + y * st;
            float v = -x * st + y * ct;
            float du = u - u_c; if(du < 0) du = -du;
            float dv = v - v_c; if(dv < 0) dv = -dv;
            // 第 2 层过滤：严格限制在方框内边界向内 INNER_OFFSET 像素的旋转矩形内
            if(du < in_hw && dv < in_hh)
            {
                if(cnt < MAX_POINTS)
                {
                    pts_x[cnt] = x;
                    pts_y[cnt] = y;
                    cnt++;
                }
                sum_x += x;
                sum_y += y;
            }
        }
    }

    if(cnt < 20) return 0;

    float mx = (float)sum_x / cnt;
    float my = (float)sum_y / cnt;

    float cov_xx = 0.0f, cov_xy = 0.0f, cov_yy = 0.0f;
    for(uint16 i = 0; i < cnt; i++)
    {
        float dx = pts_x[i] - mx;
        float dy = pts_y[i] - my;
        cov_xx += dx * dx;
        cov_xy += dx * dy;
        cov_yy += dy * dy;
    }
    cov_xx /= cnt; cov_xy /= cnt; cov_yy /= cnt;

    // PCA 主轴角度（沿 V 开口宽度方向，即两条腿端点连线方向）
    float pca_theta = 0.5f * atan2f(2.0f * cov_xy, cov_xx - cov_yy);

    // ── 融合策略：PCA 为主，方框轴作为软约束 ──
    //   理论上 V 形主轴应与方框两主轴之一平行（V 装在方框里、对齐方框）
    //   实际上 PCA 受像素噪声影响会有 ±2° 抖动，方框轴受 min_area_rect 量化误差也有 ±0.1°
    //   做加权融合：相比硬对齐，保留了 PCA 对 V 自身形状的敏感性，又借方框轴稳住基线
    //
    //   步骤 1：找方框两候选主轴（f_theta 和 f_theta+π/2）中与 PCA 主轴最接近的那个
    //   步骤 2：把 PCA 角度归一到该主轴附近（差值 diff 在 [-π/2, π/2] 内）
    //   步骤 3：v_main = frame_axis + alpha * diff
    //           alpha = 1.0 → 纯 PCA；alpha = 0.0 → 纯方框轴；这里用 0.6（PCA 权重稍大）
    float fa1 = f_theta;
    float fa2 = f_theta + PI_F * 0.5f;

    float diff1 = pca_theta - fa1;
    while(diff1 >  PI_F * 0.5f) diff1 -= PI_F;
    while(diff1 < -PI_F * 0.5f) diff1 += PI_F;

    float diff2 = pca_theta - fa2;
    while(diff2 >  PI_F * 0.5f) diff2 -= PI_F;
    while(diff2 < -PI_F * 0.5f) diff2 += PI_F;

    float closest_axis, diff;
    float ad1 = (diff1 < 0) ? -diff1 : diff1;
    float ad2 = (diff2 < 0) ? -diff2 : diff2;
    if(ad1 < ad2) { closest_axis = fa1; diff = diff1; }
    else          { closest_axis = fa2; diff = diff2; }

    // PCA 权重 0.4（可调）：偏向方框轴，让方向线更严格垂直/平行方框边
    //   ALPHA 越小 → 越贴方框轴 → 方向线越"正"（更严格和方框边垂直/平行）
    //   ALPHA 越大 → 越贴 PCA  → 方向线更"活"（对 V 自身形状更敏感）
    const float ALPHA = 0.28f;
    float v_main = closest_axis + ALPHA * diff;     // V 主轴方向（沿开口宽度）

    // ── 输出方向 = 主轴 +90°（沿 V 对称轴，即"尖端 ? 开口中心"连线方向）──
    float v_dir = v_main + PI_F * 0.5f;

    // ── 让 v_dir 指向 V 尖端：用"垂直方向宽度方差"判别（最可靠的几何法）──
    //
    //   V 形最稳健的几何特征是宽度沿对称轴变化：
    //     - 尖端侧：两条腿汇合 → 垂直对称轴方向的像素散布【窄】
    //     - 开口侧：两条腿展开 → 垂直对称轴方向的像素散布【宽】
    //   不管 V 的胖瘦/角度/线宽如何变化，这个特征是几何刚性的，绝不会反过来。
    //
    //   做法：
    //     1) 沿 v_dir 投影把 V 像素切成两半（pos 半 / neg 半）
    //     2) 各半算【垂直 v_dir 方向】的方差
    //     3) 方差小的那一半 = 尖端 → 让 v_dir 指向那一侧
    //
    //   为什么之前 pos/neg 像素数法 和 三阶矩偏度法 都失败：
    //     · pos/neg：开口/尖端像素数差异不大，二值化抖动一变就翻；
    //     · 偏度（三阶矩）：受重心位置影响，符号不稳定（开口端虽然像素多但贴近重心）；
    //     · 而"垂直宽度方差"差异能达到几十倍（尖端 ≈ 0，开口 ≈ W2），翻转阈值极高。
    float c_d   = cosf(v_dir),  s_d   = sinf(v_dir);     // 沿对称轴
    float c_p   = -s_d,         s_p   = c_d;             // 垂直对称轴

    float pos_var = 0.0f, neg_var = 0.0f;
    int32 pos_n  = 0,    neg_n  = 0;
    for(uint16 i = 0; i < cnt; i++)
    {
        float dx = pts_x[i] - mx;
        float dy = pts_y[i] - my;
        float p  = dx * c_d + dy * s_d;          // 沿对称轴投影
        float q  = dx * c_p + dy * s_p;          // 垂直对称轴投影
        if(p > 0) { pos_var += q * q; pos_n++; }
        else      { neg_var += q * q; neg_n++; }
    }
    if(pos_n > 0) pos_var /= pos_n;
    if(neg_n > 0) neg_var /= neg_n;

    // 方差小的一侧 = 尖端：
    //   pos_var < neg_var → 尖端在 +v_dir 方向 → 保持
    //   pos_var > neg_var → 尖端在 -v_dir 方向 → 反转
    if(pos_var > neg_var) v_dir += PI_F;

    // ── 输出：中心 = PCA 重心；方向 = 主轴+90° 融合后的角度（指向 V 尖端） ──
    *vx_out = (int16)mx;
    *vy_out = (int16)my;
    *vang_out = v_dir;

    return 1;
}

// ─────────────────────────────────────────
// 主搜索：BFS 找连通域 → 判方框 → 提取 V → 选最佳
// ─────────────────────────────────────────
static uint8 find_best_v(uint8 *img, v_object_t *out)
{
    memset(vis, 0, sizeof(vis));
    int8 ddx[] = {-1, 1, 0, 0};
    int8 ddy[] = {0, 0, -1, 1};

    uint16 best_score = 0;
    v_object_t best_v;
    memset(&best_v, 0, sizeof(best_v));

    for(uint16 y = 0; y < MT9V03X_H; y++)
    {
        for(uint16 x = 0; x < MT9V03X_W; x++)
        {
            if(vis[y][x] || img[y * MT9V03X_W + x] != 0) continue;

            // BFS：复用 pts_x/pts_y 作队列
            int32 qh = 0, qt = 0;
            pts_x[qt] = x; pts_y[qt] = y; qt++;
            vis[y][x] = 1;

            uint8 overflow = 0;
            while(qh < qt)
            {
                int16 cx = pts_x[qh];
                int16 cy = pts_y[qh];
                qh++;
                for(int k = 0; k < 4; k++)
                {
                    int16 nx = cx + ddx[k];
                    int16 ny = cy + ddy[k];
                    if(nx < 0 || nx >= MT9V03X_W || ny < 0 || ny >= MT9V03X_H) continue;
                    if(!vis[ny][nx] && img[ny * MT9V03X_W + nx] == 0)
                    {
                        vis[ny][nx] = 1;
                        if(qt >= MAX_POINTS) { overflow = 1; qh = qt; break; }
                        pts_x[qt] = nx;
                        pts_y[qt] = ny;
                        qt++;
                    }
                }
            }
            if(overflow) continue;
            uint16 cnt = (uint16)qt;

            // 1) 判定为方框
            int16 cor_x[4], cor_y[4];
            float f_theta, f_cx, f_cy, f_hw, f_hh;
            if(!detect_square_frame(pts_x, pts_y, cnt,
                                    cor_x, cor_y,
                                    &f_theta, &f_cx, &f_cy, &f_hw, &f_hh)) continue;

            // 2) 估计线宽（这里 cnt 是当前连通域，即"方框线"的像素数）
            float line_w = estimate_line_width(cnt, f_hw, f_hh);

            // 2.5) 把"方框线"像素在 img 中临时屏蔽（设为 128，使其不再被 PCA 当作 V 像素）
            //      这样即使 line_w 估计不精确、或方框线粗细不均，PCA 也绝不会被边线污染
            //      同时备份坐标到 frame_px/py，避免 PCA 阶段 pts_x/pts_y 被覆盖后还原失败
            for(uint16 k = 0; k < cnt; k++)
            {
                frame_px[k] = pts_x[k];
                frame_py[k] = pts_y[k];
                img[pts_y[k] * MT9V03X_W + pts_x[k]] = 128;
            }

            // 3) 在方框内部提取 V 像素，做 PCA
            int16 vx, vy;
            float vang;
            uint8 v_ok = extract_v_pca(img, f_theta, f_cx, f_cy, f_hw, f_hh, line_w,
                                       &vx, &vy, &vang);

            // 2.5) 还原 img（用备份坐标，保证 100% 还原）
            for(uint16 k = 0; k < cnt; k++)
                img[frame_py[k] * MT9V03X_W + frame_px[k]] = 0;

            if(!v_ok) continue;

            if(cnt > best_score)
            {
                best_score = cnt;
                memcpy(best_v.corner_x, cor_x, sizeof(cor_x));
                memcpy(best_v.corner_y, cor_y, sizeof(cor_y));
                best_v.cx = vx;
                best_v.cy = vy;
                best_v.angle = vang;
                best_v.valid = 1;
                best_v.score = cnt;
            }
        }
    }

    if(best_score > 0) { *out = best_v; return 1; }
    return 0;
}

// ─────────────────────────────────────────
// 坐标裁剪
// ─────────────────────────────────────────
static inline uint8 clamp_x(int16 x)
{
    if(x < 0) x = 0;
    if(x > MT9V03X_W - 1) x = MT9V03X_W - 1;
    return (uint8)x;
}
static inline uint8 clamp_y(int16 y)
{
    if(y < 0) y = 0;
    if(y > MT9V03X_H - 1) y = MT9V03X_H - 1;
    return (uint8)y;
}

// ─────────────────────────────────────────
// 用 Bresenham 把方框的 4 条边写入 xy1/yx1
// ─────────────────────────────────────────
static uint16 g_bd_idx = 0;
static void add_boundary(int16 x, int16 y)
{
    if(g_bd_idx >= BOUNDARY_NUM) return;
    xy1[g_bd_idx] = clamp_x(x);
    yx1[g_bd_idx] = clamp_y(y);
    g_bd_idx++;
}
static void draw_line_to_boundary(int16 x0, int16 y0, int16 x1, int16 y1)
{
    int16 dx = x1 - x0; if(dx < 0) dx = -dx;
    int16 dy = y1 - y0; if(dy < 0) dy = -dy;
    int16 sx = (x0 < x1) ? 1 : -1;
    int16 sy = (y0 < y1) ? 1 : -1;
    int16 err = dx - dy;
    while(1)
    {
        add_boundary(x0, y0);
        if(x0 == x1 && y0 == y1) break;
        int16 e2 = 2 * err;
        if(e2 >  -dy) { err -= dy; x0 += sx; }
        if(e2 <   dx) { err += dx; y0 += sy; }
    }
}

// ─────────────────────────────────────────
// 用 Bresenham 把方向线插值点写入 xy3/yx3
// ─────────────────────────────────────────
static uint16 g_bd3_idx = 0;
static void add_boundary3(int16 x, int16 y)
{
    if(g_bd3_idx >= BOUNDARY_NUM) return;
    xy3[g_bd3_idx] = clamp_x(x);
    yx3[g_bd3_idx] = clamp_y(y);
    g_bd3_idx++;
}
static void draw_line_to_boundary3(int16 x0, int16 y0, int16 x1, int16 y1)
{
    int16 dx = x1 - x0; if(dx < 0) dx = -dx;
    int16 dy = y1 - y0; if(dy < 0) dy = -dy;
    int16 sx = (x0 < x1) ? 1 : -1;
    int16 sy = (y0 < y1) ? 1 : -1;
    int16 err = dx - dy;
    while(1)
    {
        add_boundary3(x0, y0);
        if(x0 == x1 && y0 == y1) break;
        int16 e2 = 2 * err;
        if(e2 >  -dy) { err -= dy; x0 += sx; }
        if(e2 <   dx) { err += dx; y0 += sy; }
    }
}

// ─────────────────────────────────────────
// 把质心画成十字+中心方块，写入 xy2/yx2，让逐飞助手能清楚看到
// ─────────────────────────────────────────
static uint16 g_bd2_idx = 0;
static void add_boundary2(int16 x, int16 y)
{
    if(g_bd2_idx >= BOUNDARY_NUM) return;
    xy2[g_bd2_idx] = clamp_x(x);
    yx2[g_bd2_idx] = clamp_y(y);
    g_bd2_idx++;
}
static void draw_centroid_marker(int16 cx, int16 cy)
{
    // 横线（11 像素）
    for(int16 dx = -5; dx <= 5; dx++) add_boundary2(cx + dx, cy);
    // 竖线（11 像素）
    for(int16 dy = -5; dy <= 5; dy++) add_boundary2(cx, cy + dy);
    // 中心 3×3 实心方块，更醒目
    for(int16 dy = -1; dy <= 1; dy++)
        for(int16 dx = -1; dx <= 1; dx++)
            add_boundary2(cx + dx, cy + dy);
}

static void draw_v_result(v_object_t *v)
{
    memset(xy1, 0, sizeof(xy1));
    memset(yx1, 0, sizeof(yx1));
    memset(xy2, 0, sizeof(xy2));
    memset(yx2, 0, sizeof(yx2));
    memset(xy3, 0, sizeof(xy3));
    memset(yx3, 0, sizeof(yx3));

    if(!v->valid) return;

    // 画方框 4 条边
    g_bd_idx = 0;
    for(int i = 0; i < 4; i++)
    {
        int j = (i + 1) % 4;
        draw_line_to_boundary(v->corner_x[i], v->corner_y[i],
                              v->corner_x[j], v->corner_y[j]);
    }

    // V 质心（= 方框几何中心）—— 画十字 + 中心方块标记，清晰可见
    g_bd2_idx = 0;
    draw_centroid_marker(v->cx, v->cy);

    // V 方向线（过方框中心、与方框边垂直）—— 用 Bresenham 画整条
    float rad = v->angle;                          // 正方向 = V 尖端方向
    float cr  = cosf(rad), sr = sinf(rad);
    int16 ex  = v->cx + (int16)(LINE_LENGTH * cr); // 正方向端（V 尖端方向）
    int16 ey  = v->cy + (int16)(LINE_LENGTH * sr);
    int16 ex2 = v->cx - (int16)(LINE_LENGTH * cr); // 负方向端
    int16 ey2 = v->cy - (int16)(LINE_LENGTH * sr);

    g_bd3_idx = 0;
    draw_line_to_boundary3(ex2, ey2, ex, ey);

    // ── 正方向端画小叉（X 形）标记 V 尖端方向 ──
    //   小叉尺寸 ±4 像素，两条对角线
    int16 cx_pos = ex, cy_pos = ey;
    // 对角线 1：左上↘右下
    draw_line_to_boundary3(cx_pos - 4, cy_pos - 4, cx_pos + 4, cy_pos + 4);
    // 对角线 2：右上↙左下
    draw_line_to_boundary3(cx_pos + 4, cy_pos - 4, cx_pos - 4, cy_pos + 4);
}

// ─────────────────────────────────────────
// main
// ─────────────────────────────────────────
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();                                     // 有线 DEBUG_UART(UART_0)，图像模式下用 printf 打印坐标
#if (CAM_PRINT_MODE == PRINT_MODE_IMAGE)
    // 图像模式：本核接管无线 UART_1 发图；有线 UART_0 同时 printf 坐标（7-0 遥测须关闭）
    wireless_uart_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
#elif (CAM_PRINT_MODE == PRINT_MODE_COORD)
    // 坐标模式：本核用无线 UART_1 只发坐标，不发图（7-0 遥测须关闭）
    //   同时绑定接收回调，支持逐飞助手在线调 roll/pitch 安装补偿（全双工：边发坐标边收调参）
    wireless_uart_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
    // 先把调参字段写成默认值（与 7-0 本地默认一致），并刷入共享内存，避免 7-0 读到上电乱码
    cam_share.roll_trim  = 2.783f;
    cam_share.pitch_trim = 3.17f;
    cam_share.trim_seq   = 0;
    SCB_CleanInvalidateDCache_by_Addr(&cam_share, sizeof(cam_share));
#endif

    // 注意：P19_0 是 7-0 核 TOF(DL1B) 的软 IIC SCL 脚，此处严禁再配置为 GPIO/LED，
    //       否则推挽强推会破坏 IIC 通讯，导致 TOF 初始化失败、距离卡死在 8192。

    while(mt9v03x_init())
    {
//        gpio_toggle_level(LED1);
        system_delay_ms(500);
    }
    printf("1");
#if (CAM_PRINT_MODE == PRINT_MODE_IMAGE)
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, (uint8 *)image_copy, MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy1, xy2, xy3, yx1, yx2, yx3);
#endif
    printf("2");
    while(1)
    {
        if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            img_binarize((uint8 *)mt9v03x_image, img_bin);
            memcpy(image_copy[0], img_bin, MT9V03X_IMAGE_SIZE);

            if(find_best_v(img_bin, &target_v))
            {
#if (CAM_PRINT_MODE == PRINT_MODE_IMAGE)
                draw_v_result(&target_v);
#endif
//                gpio_set_level(LED1, 0);

                cam_share.cam_x       = target_v.cx;
                cam_share.cam_y       = target_v.cy;
                cam_share.cam_yaw_rad = target_v.angle;
                cam_share.cam_valid   = 1;
            }
            else
            {
#if (CAM_PRINT_MODE == PRINT_MODE_IMAGE)
                memset(xy1, 0, sizeof(xy1));
                memset(yx1, 0, sizeof(yx1));
                memset(xy2, 0, sizeof(xy2));
                memset(yx2, 0, sizeof(yx2));
                memset(xy3, 0, sizeof(xy3));
                memset(yx3, 0, sizeof(yx3));
#endif
//                gpio_set_level(LED1, 1);

                cam_share.cam_valid = 0;
            }
//            cam_share.cam_x       = 5;
//            cam_share.cam_y       = 5;
//            cam_share.cam_yaw_rad = 3.14f;
//            cam_share.cam_valid   = 1;
            SCB_CleanInvalidateDCache_by_Addr(&cam_share, sizeof(cam_share));

#if (CAM_PRINT_MODE == PRINT_MODE_IMAGE)
            // 图像模式：有线 UART_0 打印坐标 + 无线 UART_1 发整张图像（同时）
            //   无线发图 22560B/帧会拖慢帧率，仅调试用；参数模式下整段不执行
            printf("%d,%d,%.4f,%d\r\n",
                   cam_share.cam_x, cam_share.cam_y, cam_share.cam_yaw_rad, cam_share.cam_valid);
            seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy1, xy2, xy3, yx1, yx2, yx3);
            seekfree_assistant_camera_send();
#elif (CAM_PRINT_MODE == PRINT_MODE_COORD)
            // 坐标模式：无线 UART_1 只发坐标，不发图，无卡顿
            //   末尾附带当前 roll/pitch 补偿值 + TOF 高度回传
            //   TOF 在独立 cache line，先 Invalidate 再读，拿到 7-0 写的最新值
            SCB_CleanInvalidateDCache_by_Addr(&tof_share, sizeof(tof_share));
            {
                char coord_buf[96];
                sprintf(coord_buf, "%d,%d,%.4f,%d,%.3f,%.3f,%.3f,%d\r\n",
                        (int)cam_share.cam_x, (int)cam_share.cam_y,
                        cam_share.cam_yaw_rad, (int)cam_share.cam_valid,
                        cam_share.roll_trim, cam_share.pitch_trim,
                        tof_share.tof_height, (int)tof_share.tof_dist_mm);
                wireless_uart_send_string(coord_buf);
            }

            // 逐飞助手在线调参：收无线调参包 → 更新 roll/pitch 安装补偿
            //   通道1 = roll_trim，通道2 = pitch_trim；收到新值写入 cam_share 供 7-0 读取
            seekfree_assistant_data_analysis();
            {
                uint8 trim_updated = 0;
                if(seekfree_assistant_parameter_update_flag[0])
                {
                    cam_share.roll_trim = seekfree_assistant_parameter[0];
                    seekfree_assistant_parameter_update_flag[0] = 0;
                    trim_updated = 1;
                }
                if(seekfree_assistant_parameter_update_flag[1])
                {
                    cam_share.pitch_trim = seekfree_assistant_parameter[1];
                    seekfree_assistant_parameter_update_flag[1] = 0;
                    trim_updated = 1;
                }
                if(trim_updated)
                {
                    cam_share.trim_seq++;     // 通知 7-0 有新补偿值
                    SCB_CleanInvalidateDCache_by_Addr(&cam_share, sizeof(cam_share));
                }
            }
#endif
            // 参数模式：本核静默，只算 cam_share 供飞控用，不发图也不打印
        }
    }
}
