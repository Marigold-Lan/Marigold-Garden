using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace FlowField.Jobs
{
    /// <summary>
    /// 目标点标记Job - 将目标区域标记到网格
    /// </summary>
    [BurstCompile]
    public struct MarkGoalsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float2> GoalPositions;
        [ReadOnly] public NativeArray<float> GoalRadii;
        [ReadOnly] public NativeArray<int> GoalPriorities;
        [ReadOnly] public int GoalCount;

        [WriteOnly] public NativeArray<byte> CellTypes;
        [WriteOnly] public NativeArray<float> GoalDistances;

        public int2 GridSize;
        public float CellSize;
        public float2 Origin;

        public void Execute(int cellIndex)
        {
            int x = cellIndex % GridSize.x;
            int y = cellIndex / GridSize.x;

            float2 cellCenter = Origin + new float2(x + 0.5f, y + 0.5f) * CellSize;

            bool hasGoal = false;
            float minScore = float.MaxValue;

            for (int i = 0; i < GoalCount; i++)
            {
                float dist = math.length(cellCenter - GoalPositions[i]);

                if (dist <= GoalRadii[i])
                {
                    // 在目标区域内
                    CellTypes[cellIndex] = (byte)CellType.Goal;
                    GoalDistances[cellIndex] = 0f;
                    return;
                }

                // 计算评分（距离/优先级）
                float score = dist / math.max(1, GoalPriorities[i]);
                if (score < minScore)
                {
                    minScore = score;
                    hasGoal = true;
                }
            }

            if (hasGoal)
            {
                // 保持Passable状态，但标记Goal距离
                GoalDistances[cellIndex] = minScore;
            }
        }
    }

    /// <summary>
    /// Goal距离传播Job - BFS式波前传播
    /// 使用多目标竞争机制，选择最近的目标
    /// </summary>
    [BurstCompile]
    public struct GoalDistancePropagationJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float> OldGoalDistances;
        [ReadOnly] public NativeArray<byte> CellTypes;

        [WriteOnly] public NativeArray<float> NewGoalDistances;
        [WriteOnly] public NativeArray<int> Integrated;

        public int2 GridSize;
        public int Pass;
        public int PassIndex;

        public void Execute(int index)
        {
            int x = index % GridSize.x;
            int y = index / GridSize.x;

            // 如果是目标点，距离为0
            if (CellTypes[index] == (byte)CellType.Goal)
            {
                NewGoalDistances[index] = 0f;
                Integrated[index] = PassIndex;
                return;
            }

            // 如果已经收敛，跳过
            if (Pass > 0 && Integrated[index] == PassIndex)
            {
                NewGoalDistances[index] = OldGoalDistances[index];
                return;
            }

            float bestDist = OldGoalDistances[index];

            // 4方向邻居（卡洪移动）
            int dx0 = -1, dy0 = 0;
            int dx1 =  1, dy1 = 0;
            int dx2 =  0, dy2 = -1;
            int dx3 =  0, dy3 = 1;

            for (int i = 0; i < 4; i++)
            {
                int ddx = i == 0 ? dx0 : i == 1 ? dx1 : i == 2 ? dx2 : dx3;
                int ddy = i == 0 ? dy0 : i == 1 ? dy1 : i == 2 ? dy2 : dy3;
                int nx = x + ddx;
                int ny = y + ddy;

                if (nx < 0 || nx >= GridSize.x || ny < 0 || ny >= GridSize.y)
                    continue;

                int neighborIdx = ny * GridSize.x + nx;

                if (CellTypes[neighborIdx] == (byte)CellType.Obstacle)
                    continue;

                float neighborDist = OldGoalDistances[neighborIdx];
                if (neighborDist < float.MaxValue)
                {
                    float pathDist = neighborDist + 1f;
                    if (pathDist < bestDist)
                    {
                        bestDist = pathDist;
                    }
                }
            }

            NewGoalDistances[index] = bestDist;

            // 如果这一轮更新了，标记
            if (bestDist < OldGoalDistances[index])
            {
                Integrated[index] = PassIndex;
            }
        }
    }

    /// <summary>
    /// 目标距离初始化Job
    /// </summary>
    [BurstCompile]
    public struct GoalDistanceInitJob : IJobParallelFor
    {
        [WriteOnly] public NativeArray<float> GoalDistances;
        [WriteOnly] public NativeArray<int> Integrated;

        public int TotalCells;
        public float InitValue;

        public void Execute(int index)
        {
            GoalDistances[index] = InitValue;
            Integrated[index] = -1;
        }
    }

    /// <summary>
    /// 流场方向计算Job - 基于Goal距离梯度计算方向
    /// Direction = -grad(GoalDistances)
    /// </summary>
    [BurstCompile]
    public struct FlowDirectionJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float> GoalDistances;
        [ReadOnly] public NativeArray<byte> CellTypes;

        [WriteOnly] public NativeArray<float2> Directions;

        public int2 GridSize;

        public void Execute(int index)
        {
            int x = index % GridSize.x;
            int y = index / GridSize.x;

            // 如果是障碍物，方向为(0,0)
            if (CellTypes[index] == (byte)CellType.Obstacle)
            {
                Directions[index] = float2.zero;
                return;
            }

            // 计算梯度
            float dist = GoalDistances[index];

            float dx = 0f;
            float dy = 0f;

            // 右侧邻居
            if (x + 1 < GridSize.x)
            {
                int rightIdx = y * GridSize.x + (x + 1);
                if (CellTypes[rightIdx] != (byte)CellType.Obstacle)
                {
                    dx = GoalDistances[rightIdx] - dist;
                }
            }

            // 左侧邻居
            if (x - 1 >= 0)
            {
                int leftIdx = y * GridSize.x + (x - 1);
                if (CellTypes[leftIdx] != (byte)CellType.Obstacle)
                {
                    dx = math.min(dx, dist - GoalDistances[leftIdx]);
                }
            }

            // 上方邻居
            if (y + 1 < GridSize.y)
            {
                int upIdx = (y + 1) * GridSize.x + x;
                if (CellTypes[upIdx] != (byte)CellType.Obstacle)
                {
                    dy = GoalDistances[upIdx] - dist;
                }
            }

            // 下方邻居
            if (y - 1 >= 0)
            {
                int downIdx = (y - 1) * GridSize.x + x;
                if (CellTypes[downIdx] != (byte)CellType.Obstacle)
                {
                    dy = math.min(dy, dist - GoalDistances[downIdx]);
                }
            }

            // 梯度指向远离目标的方向，所以取负
            float2 gradient = new float2(-dx, -dy);
            float lenSq = math.lengthsq(gradient);

            if (lenSq < 1e-10f)
            {
                Directions[index] = float2.zero;
            }
            else
            {
                Directions[index] = gradient / math.sqrt(lenSq);
            }
        }
    }

    /// <summary>
    /// 多目标流场方向Job - 处理多个目标点竞争的情况
    /// </summary>
    [BurstCompile]
    public struct MultiGoalFlowDirectionJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float> GoalDistances;
        [ReadOnly] public NativeArray<byte> CellTypes;
        [ReadOnly] public NativeArray<int> GoalAssignments; // 每个格子归属哪个目标

        [WriteOnly] public NativeArray<float2> Directions;

        public int2 GridSize;

        public void Execute(int index)
        {
            int x = index % GridSize.x;
            int y = index / GridSize.x;

            if (CellTypes[index] == (byte)CellType.Obstacle)
            {
                Directions[index] = float2.zero;
                return;
            }

            if (CellTypes[index] == (byte)CellType.Goal)
            {
                Directions[index] = float2.zero;
                return;
            }

            float dist = GoalDistances[index];

            // 检查4个邻居
            float bestDx = 0f, bestDy = 0f;
            float bestDist = dist;

            // 4方向邻居
            if (x + 1 < GridSize.x)
            {
                int rightIdx = y * GridSize.x + (x + 1);
                if (CellTypes[rightIdx] != (byte)CellType.Obstacle)
                {
                    float neighborDist = GoalDistances[rightIdx];
                    if (neighborDist < bestDist)
                    {
                        bestDist = neighborDist;
                        bestDx = 1f;
                        bestDy = 0f;
                    }
                }
            }

            if (x - 1 >= 0)
            {
                int leftIdx = y * GridSize.x + (x - 1);
                if (CellTypes[leftIdx] != (byte)CellType.Obstacle)
                {
                    float neighborDist = GoalDistances[leftIdx];
                    if (neighborDist < bestDist)
                    {
                        bestDist = neighborDist;
                        bestDx = -1f;
                        bestDy = 0f;
                    }
                }
            }

            if (y + 1 < GridSize.y)
            {
                int upIdx = (y + 1) * GridSize.x + x;
                if (CellTypes[upIdx] != (byte)CellType.Obstacle)
                {
                    float neighborDist = GoalDistances[upIdx];
                    if (neighborDist < bestDist)
                    {
                        bestDist = neighborDist;
                        bestDx = 0f;
                        bestDy = 1f;
                    }
                }
            }

            if (y - 1 >= 0)
            {
                int downIdx = (y - 1) * GridSize.x + x;
                if (CellTypes[downIdx] != (byte)CellType.Obstacle)
                {
                    float neighborDist = GoalDistances[downIdx];
                    if (neighborDist < bestDist)
                    {
                        bestDist = neighborDist;
                        bestDx = 0f;
                        bestDy = -1f;
                    }
                }
            }

            float2 direction = new float2(bestDx, bestDy);
            float lenSq = math.lengthsq(direction);

            if (lenSq < 1e-10f)
            {
                Directions[index] = float2.zero;
            }
            else
            {
                Directions[index] = direction / math.sqrt(lenSq);
            }
        }
    }

    /// <summary>
    /// 方向平滑Job - 在方向场中创建平滑过渡
    /// </summary>
    [BurstCompile]
    public struct DirectionSmoothJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float2> InputDirections;
        [ReadOnly] public NativeArray<byte> CellTypes;

        [WriteOnly] public NativeArray<float2> OutputDirections;

        public int2 GridSize;

        public void Execute(int index)
        {
            int x = index % GridSize.x;
            int y = index / GridSize.x;

            if (CellTypes[index] == (byte)CellType.Obstacle)
            {
                OutputDirections[index] = float2.zero;
                return;
            }

            float2 sum = InputDirections[index];
            float weight = 1f;

            // 4方向邻居 - 内联偏移
            if (x > 0)
            {
                int neighborIdx = y * GridSize.x + (x - 1);
                if (CellTypes[neighborIdx] != (byte)CellType.Obstacle) { sum += InputDirections[neighborIdx]; weight += 1f; }
            }
            if (x < GridSize.x - 1)
            {
                int neighborIdx = y * GridSize.x + (x + 1);
                if (CellTypes[neighborIdx] != (byte)CellType.Obstacle) { sum += InputDirections[neighborIdx]; weight += 1f; }
            }
            if (y > 0)
            {
                int neighborIdx = (y - 1) * GridSize.x + x;
                if (CellTypes[neighborIdx] != (byte)CellType.Obstacle) { sum += InputDirections[neighborIdx]; weight += 1f; }
            }
            if (y < GridSize.y - 1)
            {
                int neighborIdx = (y + 1) * GridSize.x + x;
                if (CellTypes[neighborIdx] != (byte)CellType.Obstacle) { sum += InputDirections[neighborIdx]; weight += 1f; }
            }

            float2 avg = sum / weight;
            float lenSq = math.lengthsq(avg);

            if (lenSq < 1e-10f)
            {
                OutputDirections[index] = float2.zero;
            }
            else
            {
                OutputDirections[index] = avg / math.sqrt(lenSq);
            }
        }
    }
}