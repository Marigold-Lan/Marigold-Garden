using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace FlowField.Jobs
{
    /// <summary>
    /// 跳步算法（JFA - Jump Flood Algorithm）计算有符号距离场
    ///
    /// 算法原理：
    /// 1. 初始化：障碍物单元格标记为0，边界外标记为大正数
    /// 2. 迭代：每轮使用2^k步长的探测
    /// 3. 收敛：约log2(max(Width, Height))轮迭代后收敛
    ///
    /// 时间复杂度: O(N × log(D)) 其中N为像素数，D为最大距离
    /// 空间复杂度: O(N)
    /// </summary>
    [BurstCompile]
    public struct JFA_SDFInitJob : IJobParallelFor
    {
        [WriteOnly] public NativeArray<float> Distances;
        [WriteOnly] public NativeArray<byte> CellTypes;

        public int TotalCells;
        public float MaxDistance;

        public void Execute(int index)
        {
            CellTypes[index] = (byte)CellType.Passable;
            Distances[index] = MaxDistance;
        }
    }

    /// <summary>
    /// JFA单轮迭代Job
    /// </summary>
    [BurstCompile]
    public struct JFA_SDFIterationJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float> InputDistances;
        [ReadOnly] public NativeArray<byte> CellTypes;

        [WriteOnly] public NativeArray<float> OutputDistances;

        public int2 GridSize;
        public int Step;
        public int Pass;

        public void Execute(int index)
        {
            int x = index % GridSize.x;
            int y = index / GridSize.x;

            float currentDist = InputDistances[index];

            // 如果已经是边界点，跳过
            if (CellTypes[index] == (byte)CellType.Obstacle)
            {
                OutputDistances[index] = 0f;
                return;
            }

            float bestDist = currentDist;

            // 8方向探测 - 展开为固定偏移
            int s = Step;
            ProbeNeighbor(x, y, -s,  0, ref bestDist);
            ProbeNeighbor(x, y,  s,  0, ref bestDist);
            ProbeNeighbor(x, y,  0, -s, ref bestDist);
            ProbeNeighbor(x, y,  0,  s, ref bestDist);
            ProbeNeighbor(x, y, -s, -s, ref bestDist);
            ProbeNeighbor(x, y,  s, -s, ref bestDist);
            ProbeNeighbor(x, y, -s,  s, ref bestDist);
            ProbeNeighbor(x, y,  s,  s, ref bestDist);

            OutputDistances[index] = bestDist;
        }

        [GenerateTestsForBurstCompatibility]
        private void ProbeNeighbor(int x, int y, int ddx, int ddy, ref float bestDist)
        {
            int nx = x + ddx;
            int ny = y + ddy;
            if (nx < 0 || nx >= GridSize.x || ny < 0 || ny >= GridSize.y)
                return;

            int neighborIdx = ny * GridSize.x + nx;
            float neighborDist = InputDistances[neighborIdx];

            if (CellTypes[neighborIdx] == (byte)CellType.Obstacle)
            {
                float adx = math.abs(ddx);
                float ady = math.abs(ddy);
                bestDist = math.min(bestDist, (adx + ady) * 0.707f);
            }
            else if (neighborDist < float.MaxValue)
            {
                float adx = math.abs(ddx);
                float ady = math.abs(ddy);
                bestDist = math.min(bestDist, neighborDist + (adx + ady) * 0.707f);
            }
        }
    }

    /// <summary>
    /// 障碍物标记Job - 将障碍物区域标记到网格
    /// </summary>
    [BurstCompile]
    public struct MarkObstaclesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float2> ObstaclePositions;
        [ReadOnly] public NativeArray<float> ObstacleRadii;
        [ReadOnly] public NativeArray<byte> ObstacleShapes; // 0=circle, 1=box
        [ReadOnly] public NativeArray<float2> ObstacleHalfExtents; // for boxes
        [ReadOnly] public int ObstacleCount;

        [WriteOnly] public NativeArray<byte> CellTypes;
        [WriteOnly] public NativeArray<float> Distances;

        public int2 GridSize;
        public float CellSize;
        public float2 Origin;
        public float MaxDistance;
        public float2 BoundsMin;
        public float2 BoundsMax;

        public void Execute(int cellIndex)
        {
            int x = cellIndex % GridSize.x;
            int y = cellIndex / GridSize.x;

            float2 cellCenter = Origin + new float2(x + 0.5f, y + 0.5f) * CellSize;

            float minSDF = MaxDistance;

            for (int i = 0; i < ObstacleCount; i++)
            {
                float sdf;
                if (ObstacleShapes[i] == 0) // Circle
                {
                    sdf = FlowFieldMath.SDFCircle(cellCenter, ObstaclePositions[i], ObstacleRadii[i]);
                }
                else // Box
                {
                    sdf = FlowFieldMath.SDFBox(cellCenter, ObstaclePositions[i], ObstacleHalfExtents[i]);
                }

                if (sdf < 0f)
                {
                    // 在障碍物内部
                    CellTypes[cellIndex] = (byte)CellType.Obstacle;
                    Distances[cellIndex] = 0f;
                    return;
                }

                minSDF = math.min(minSDF, sdf);
            }

            // 将SDF转换为单元格距离
            float cellDist = minSDF / CellSize;
            Distances[cellIndex] = cellDist;
        }
    }

    /// <summary>
    /// 增量SDF更新Job - 只更新受影响的区域
    /// </summary>
    [BurstCompile]
    public struct IncrementalSDFUpdateJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float> OldDistances;
        [ReadOnly] public NativeArray<byte> CellTypes;

        [WriteOnly] public NativeArray<float> NewDistances;

        public int2 GridSize;
        public int2 DirtyMin;
        public int2 DirtyMax;
        public float CellSize;

        public void Execute(int cellIndex)
        {
            int x = cellIndex % GridSize.x;
            int y = cellIndex / GridSize.x;

            // 如果不在脏区域，直接复制旧值
            if (x < DirtyMin.x || x > DirtyMax.x || y < DirtyMin.y || y > DirtyMax.y)
            {
                NewDistances[cellIndex] = OldDistances[cellIndex];
                return;
            }

            // 如果是障碍物，距离为0
            if (CellTypes[cellIndex] == (byte)CellType.Obstacle)
            {
                NewDistances[cellIndex] = 0f;
                return;
            }

            // 重新计算距离
            float bestDist = float.MaxValue;
            int2 center = new int2(x, y);

            int searchRadius = 5;
            for (int dy = -searchRadius; dy <= searchRadius; dy++)
            {
                for (int dx = -searchRadius; dx <= searchRadius; dx++)
                {
                    int nx = x + dx;
                    int ny = y + dy;

                    if (nx < 0 || nx >= GridSize.x || ny < 0 || ny >= GridSize.y)
                        continue;

                    int neighborIdx = ny * GridSize.x + nx;

                    if (CellTypes[neighborIdx] == (byte)CellType.Obstacle)
                    {
                        int dist = math.max(math.abs(dx), math.abs(dy));
                        bestDist = math.min(bestDist, dist);
                    }
                }
            }

            NewDistances[cellIndex] = bestDist;
        }
    }

    /// <summary>
    /// SDF边界平滑Job - 在障碍物边界创建平滑过渡
    /// </summary>
    [BurstCompile]
    public struct SDFSmoothJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float> InputDistances;
        [ReadOnly] public NativeArray<byte> CellTypes;

        [WriteOnly] public NativeArray<float> OutputDistances;

        public int2 GridSize;
        public float SmoothFactor;
        public float BoundaryThreshold;

        public void Execute(int index)
        {
            int x = index % GridSize.x;
            int y = index / GridSize.x;

            float centerDist = InputDistances[index];

            // 只对边界区域进行平滑
            if (centerDist > BoundaryThreshold || centerDist <= 0f)
            {
                OutputDistances[index] = centerDist;
                return;
            }

            float sum = centerDist;
            float weight = 1f;

            // 4方向邻居 - 内联偏移
            if (x > 0) { sum += InputDistances[y * GridSize.x + (x - 1)]; weight += 1f; }
            if (x < GridSize.x - 1) { sum += InputDistances[y * GridSize.x + (x + 1)]; weight += 1f; }
            if (y > 0) { sum += InputDistances[(y - 1) * GridSize.x + x]; weight += 1f; }
            if (y < GridSize.y - 1) { sum += InputDistances[(y + 1) * GridSize.x + x]; weight += 1f; }

            OutputDistances[index] = math.lerp(centerDist, sum / weight, SmoothFactor);
        }
    }
}