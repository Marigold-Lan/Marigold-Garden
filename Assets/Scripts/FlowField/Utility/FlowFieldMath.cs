using Unity.Mathematics;

namespace FlowField
{
    /// <summary>
    /// 流场计算专用的数学工具函数
    /// 所有函数都标记为[BurstCompatible]以支持Burst编译
    /// </summary>
    public static class FlowFieldMath
    {
        // ==================== 向量运算 ====================

        /// <summary>
        /// 安全归一化2D向量，处理零向量
        /// </summary>
        [BurstCompatible]
        public static float2 NormalizeSafe(float2 v, float2 fallback = default)
        {
            float lenSq = math.lengthsq(v);
            if (lenSq < 1e-10f)
                return fallback;
            return v / math.sqrt(lenSq);
        }

        /// <summary>
        /// 计算两点间距离的平方（避免开方）
        /// </summary>
        [BurstCompatible]
        public static float DistanceSq(float2 a, float2 b)
        {
            float2 diff = a - b;
            return math.lengthsq(diff);
        }

        /// <summary>
        /// 计算两点间距离
        /// </summary>
        [BurstCompatible]
        public static float Distance(float2 a, float2 b)
        {
            float2 diff = a - b;
            return math.sqrt(math.lengthsq(diff));
        }

        /// <summary>
        /// 快速平方距离（整数坐标）
        /// </summary>
        [BurstCompatible]
        public static int DistanceSqInt(int2 a, int2 b)
        {
            int dx = a.x - b.x;
            int dy = a.y - b.y;
            return dx * dx + dy * dy;
        }

        /// <summary>
        /// 整数坐标的Chebyshev距离
        /// </summary>
        [BurstCompatible]
        public static int ChebyshevDistance(int2 a, int2 b)
        {
            return math.max(math.abs(a.x - b.x), math.abs(a.y - b.y));
        }

        /// <summary>
        /// 整数坐标的Manhattan距离
        /// </summary>
        [BurstCompatible]
        public static int ManhattanDistance(int2 a, int2 b)
        {
            return math.abs(a.x - b.x) + math.abs(a.y - b.y);
        }

        // ==================== SDF相关 ====================

        /// <summary>
        /// 计算圆形的SDF（外部为正，内部为负）
        /// </summary>
        [BurstCompatible]
        public static float SDFCircle(float2 point, float2 center, float radius)
        {
            return math.distance(point, center) - radius;
        }

        /// <summary>
        /// 计算轴对齐矩形的SDF
        /// </summary>
        [BurstCompatible]
        public static float SDFBox(float2 point, float2 center, float2 halfExtents)
        {
            float2 q = math.abs(point - center) - halfExtents;
            float2 qMax = math.max(q, float2.zero);
            return math.length(qMax) + math.min(math.max(q.x, q.y), 0.0f);
        }

        /// <summary>
        /// 圆形SDF的梯度（用于流场方向）
        /// </summary>
        [BurstCompatible]
        public static float2 SDFCircleGradient(float2 point, float2 center)
        {
            float2 diff = point - center;
            float dist = math.length(diff);
            if (dist < 1e-6f)
                return new float2(0.0f, 1.0f);
            return diff / dist;
        }

        /// <summary>
        /// 盒子的SDF梯度
        /// </summary>
        [BurstCompatible]
        public static float2 SDFBoxGradient(float2 point, float2 center, float2 halfExtents)
        {
            float2 q = math.abs(point - center) - halfExtents;

            if (q.x > q.y)
                return new float2(math.sign(point.x - center.x), 0.0f);
            else
                return new float2(0.0f, math.sign(point.y - center.y));
        }

        /// <summary>
        /// SDF并集运算
        /// </summary>
        [BurstCompatible]
        public static float SDFUnion(float d1, float d2)
        {
            return math.min(d1, d2);
        }

        /// <summary>
        /// SDF交集运算
        /// </summary>
        [BurstCompatible]
        public static float SDFIntersect(float d1, float d2)
        {
            return math.max(d1, d2);
        }

        /// <summary>
        /// SDF差集运算
        /// </summary>
        [BurstCompatible]
        public static float SDFDifference(float d1, float d2)
        {
            return math.max(d1, -d2);
        }

        /// <summary>
        /// SDF平滑并集
        /// </summary>
        [BurstCompatible]
        public static float SDFSmoothUnion(float d1, float d2, float k)
        {
            float h = math.saturate(0.5f + 0.5f * (d2 - d1) / k);
            return math.lerp(d2, d1, h) - k * h * (1.0f - h);
        }

        // ==================== 网格遍历 ====================

        /// <summary>
        /// 获取8方向邻居偏移
        /// </summary>
        [BurstCompatible]
        public static readonly int2[] RingOffsets8 = new int2[]
        {
            new int2(-1,  0), new int2( 1,  0),
            new int2( 0, -1), new int2( 0,  1),
            new int2(-1, -1), new int2( 1, -1),
            new int2(-1,  1), new int2( 1,  1),
        };

        /// <summary>
        /// 获取4方向邻居偏移（卡洪移动）
        /// </summary>
        [BurstCompatible]
        public static readonly int2[] RingOffsets4 = new int2[]
        {
            new int2(-1,  0), new int2( 1,  0),
            new int2( 0, -1), new int2( 0,  1),
        };

        /// <summary>
        /// 获取8方向邻居偏移（带权重）
        /// </summary>
        [BurstCompatible]
        public static readonly float2[] RingOffsets8Weighted = new float2[]
        {
            new float2(-1,  0), new float2( 1,  0),
            new float2( 0, -1), new float2( 0,  1),
            new float2(-1, -1) * 0.707f, new float2( 1, -1) * 0.707f,
            new float2(-1,  1) * 0.707f, new float2( 1,  1) * 0.707f,
        };

        /// <summary>
        /// 获取指定半径内的所有单元格
        /// </summary>
        [BurstCompatible]
        public static void GetCellsInRadius(int2 center, float radius, float cellSize, NativeList<int2> cells)
        {
            int radiusCells = (int)math.ceil(radius / cellSize);

            cells.Clear();
            for (int y = -radiusCells; y <= radiusCells; y++)
            {
                for (int x = -radiusCells; x <= radiusCells; x++)
                {
                    int2 cell = center + new int2(x, y);
                    if (ChebyshevDistance(center, cell) <= radiusCells)
                    {
                        cells.Add(cell);
                    }
                }
            }
        }

        // ==================== 插值和平滑 ====================

        /// <summary>
        /// 双线性插值
        /// </summary>
        [BurstCompatible]
        public static float Bilerp(float v00, float v10, float v01, float v11, float2 t)
        {
            float x0 = math.lerp(v00, v10, t.x);
            float x1 = math.lerp(v01, v11, t.x);
            return math.lerp(x0, x1, t.y);
        }

        /// <summary>
        /// 平滑阶梯函数
        /// </summary>
        [BurstCompatible]
        public static float SmoothStep(float edge0, float edge1, float value)
        {
            float t = math.saturate((value - edge0) / (edge1 - edge0));
            return t * t * (3.0f - 2.0f * t);
        }

        /// <summary>
        /// 平滑阻尼
        /// </summary>
        [BurstCompatible]
        public static float SmoothDamp(float current, float target, ref float velocity, float smoothTime, float deltaTime, float maxSpeed = float.MaxValue)
        {
            smoothTime = math.max(0.0001f, smoothTime);
            float omega = 2.0f / smoothTime;
            float x = omega * deltaTime;
            float exp = 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
            float change = current - target;
            float temp = (change + maxSpeed * smoothTime) * omega * exp;

            float output = target + (change - maxSpeed * smoothTime) * exp + maxSpeed * smoothTime - temp;

            if (target - current > maxSpeed * smoothTime)
            {
                output = math.max(output, target - maxSpeed * smoothTime);
            }

            return output;
        }

        /// <summary>
        /// 角度插值（最短路径）
        /// </summary>
        [BurstCompatible]
        public static float LerpAngle(float a, float b, float t)
        {
            float delta = math.atan2(math.sin(b - a), math.cos(b - a));
            return a + delta * t;
        }

        /// <summary>
        /// 将角度归一化到[-PI, PI]
        /// </summary>
        [BurstCompatible]
        public static float NormalizeAngle(float angle)
        {
            while (angle > math.PI) angle -= 2 * math.PI;
            while (angle < -math.PI) angle += 2 * math.PI;
            return angle;
        }

        // ==================== 角度与向量 ====================

        /// <summary>
        /// 弧度转方向向量
        /// </summary>
        [BurstCompatible]
        public static float2 AngleToDirection(float angle)
        {
            return new float2(math.cos(angle), math.sin(angle));
        }

        /// <summary>
        /// 方向向量转弧度
        /// </summary>
        [BurstCompatible]
        public static float DirectionToAngle(float2 dir)
        {
            return math.atan2(dir.y, dir.x);
        }

        /// <summary>
        /// 限制向量长度
        /// </summary>
        [BurstCompatible]
        public static float2 ClampMagnitude(float2 v, float maxLength)
        {
            float lenSq = math.lengthsq(v);
            if (lenSq > maxLength * maxLength)
            {
                return v * (maxLength / math.sqrt(lenSq));
            }
            return v;
        }

        /// <summary>
        /// 点到线段的距离
        /// </summary>
        [BurstCompatible]
        public static float PointToSegmentDistance(float2 point, float2 segStart, float2 segEnd)
        {
            float2 offset = segEnd - segStart;
            float t = math.saturate(math.dot(point - segStart, offset) / math.lengthsq(offset));
            float2 projection = segStart + t * offset;
            return math.distance(point, projection);
        }

        // ==================== 哈希函数 ====================

        /// <summary>
        /// Wang hash - 高质量低碰撞哈希函数
        /// </summary>
        [BurstCompatible]
        public static uint WangHash(uint key)
        {
            key = (key ^ 61) ^ (key >> 16);
            key = key + (key << 3);
            key = key ^ (key >> 4);
            key = key * 0x27d4eb2d;
            key = key ^ (key >> 15);
            return key;
        }

        /// <summary>
        /// 2D坐标哈希
        /// </summary>
        [BurstCompatible]
        public static int HashCell(int2 cell, int bucketCount)
        {
            uint hash = (uint)cell.x;
            hash = (hash * 0x45d9f3bU) ^ (uint)cell.y;
            hash = (hash * 0x45d9f3bU) ^ (uint)(cell.x >> 16);
            hash = (hash * 0x45d9f3bU) ^ (uint)(cell.y >> 16);
            return (int)(hash % (uint)bucketCount);
        }
    }
}