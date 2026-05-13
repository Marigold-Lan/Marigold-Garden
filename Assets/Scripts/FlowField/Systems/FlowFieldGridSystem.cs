using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace FlowField.Systems
{
    /// <summary>
    /// 流场网格系统 - 管理流场网格的更新
    /// 包括SDF计算、目标距离传播、方向场生成
    /// </summary>
    public partial class FlowFieldGridSystem : SystemBase
    {
        private FlowFieldGrid grid;
        private int2 gridSize;
        private float cellSize;
        private float2 origin;
        private int version;
        private int sdfVersion;
        private int directionVersion;

        protected override void OnCreate()
        {
            // 默认配置：100x100网格，1单位单元格大小
            gridSize = new int2(100, 100);
            cellSize = 1f;
            origin = new float2(-50f, -50f);

            grid = FlowFieldGrid.Create(gridSize, cellSize, origin, Allocator.Persistent);
            version = 0;
        }

        protected override void OnDestroy()
        {
            if (grid.Distances.IsCreated)
            {
                grid.Dispose();
            }
        }

        protected override void OnUpdate()
        {
            // 如果没有障碍物或目标变化，跳过更新
            bool needsUpdate = false;

            // 检查是否有障碍物更新请求
            foreach (var obstacle in SystemAPI.Query<RefRO<FlowFieldObstacle>>().WithChangeFilter<FlowFieldObstacle>())
            {
                needsUpdate = true;
                break;
            }

            // 检查是否有目标更新请求
            foreach (var goal in SystemAPI.Query<RefRO<FlowFieldGoal>>().WithChangeFilter<FlowFieldGoal>())
            {
                needsUpdate = true;
                break;
            }

            if (!needsUpdate) return;

            // 收集所有障碍物
            NativeList<float2> obstaclePositions = new NativeList<float2>(256, Allocator.TempJob);
            NativeList<float> obstacleRadii = new NativeList<float>(256, Allocator.TempJob);
            NativeList<byte> obstacleShapes = new NativeList<byte>(256, Allocator.TempJob);
            NativeList<float2> obstacleHalfExtents = new NativeList<float2>(256, Allocator.TempJob);

            foreach (var obstacle in SystemAPI.Query<RefRO<FlowFieldObstacle>>())
            {
                obstaclePositions.Add(obstacle.ValueRO.Position);
                obstacleRadii.Add(obstacle.ValueRO.Radius);
                obstacleShapes.Add((byte)obstacle.ValueRO.Shape);
                obstacleHalfExtents.Add(obstacle.ValueRO.HalfExtents);
            }

            // 收集所有目标
            NativeList<float2> goalPositions = new NativeList<float2>(64, Allocator.TempJob);
            NativeList<float> goalRadii = new NativeList<float>(64, Allocator.TempJob);
            NativeList<int> goalPriorities = new NativeList<int>(64, Allocator.TempJob);

            foreach (var goal in SystemAPI.Query<RefRO<FlowFieldGoal>>())
            {
                if (goal.ValueRO.IsActive)
                {
                    goalPositions.Add(goal.ValueRO.Position);
                    goalRadii.Add(goal.ValueRO.Radius);
                    goalPriorities.Add(goal.ValueRO.Priority);
                }
            }

            // 1. 标记障碍物
            var markObstaclesJob = new MarkObstaclesGridJob
            {
                ObstaclePositions = obstaclePositions.AsArray(),
                ObstacleRadii = obstacleRadii.AsArray(),
                ObstacleShapes = obstacleShapes.AsArray(),
                ObstacleHalfExtents = obstacleHalfExtents.AsArray(),
                CellTypes = grid.CellTypes,
                Distances = grid.Distances,
                GridSize = gridSize,
                CellSize = cellSize,
                Origin = origin,
                MaxDistance = math.max(gridSize.x, gridSize.y)
            };
            markObstaclesJob.Run();

            // 2. 标记目标
            var markGoalsJob = new MarkGoalsGridJob
            {
                GoalPositions = goalPositions.AsArray(),
                GoalRadii = goalRadii.AsArray(),
                GoalPriorities = goalPriorities.AsArray(),
                CellTypes = grid.CellTypes,
                GoalDistances = grid.GoalDistances,
                GridSize = gridSize,
                CellSize = cellSize,
                Origin = origin
            };
            markGoalsJob.Run();

            // 3. 初始化目标距离
            var initGoalDistJob = new InitGoalDistancesJob
            {
                GoalDistances = grid.GoalDistances,
                Integrated = grid.GoalIntegrated,
                TotalCells = grid.TotalCells,
                InitValue = float.MaxValue
            };
            initGoalDistJob.Run();

            // 4. BFS传播目标距离
            int maxPasses = math.max(gridSize.x, gridSize.y);
            NativeArray<float> tempGoalDist = new NativeArray<float>(grid.TotalCells, Allocator.TempJob);

            for (int pass = 0; pass < maxPasses; pass++)
            {
                bool anyUpdated = false;

                var propagationJob = new PropagateGoalDistancesJob
                {
                    InputDistances = pass == 0 ? grid.GoalDistances : tempGoalDist,
                    OutputDistances = tempGoalDist,
                    CellTypes = grid.CellTypes,
                    Integrated = grid.GoalIntegrated,
                    GridSize = gridSize,
                    PassIndex = pass
                };

                propagationJob.Run();

                // 复制结果
                NativeArray<float> src = pass == 0 ? tempGoalDist : grid.GoalDistances;
                NativeArray<float> dst = pass == 0 ? grid.GoalDistances : tempGoalDist;
                tempGoalDist.CopyFrom(src);

                // 检查是否有更新
                for (int i = 0; i < grid.TotalCells; i++)
                {
                    if (tempGoalDist[i] < grid.GoalDistances[i])
                    {
                        anyUpdated = true;
                        break;
                    }
                }

                if (!anyUpdated) break;
            }

            // 5. 计算流场方向
            var directionJob = new ComputeFlowDirectionsJob
            {
                GoalDistances = grid.GoalDistances,
                CellTypes = grid.CellTypes,
                Directions = grid.Directions,
                GridSize = gridSize
            };
            directionJob.Run();

            // 清理
            obstaclePositions.Dispose();
            obstacleRadii.Dispose();
            obstacleShapes.Dispose();
            obstacleHalfExtents.Dispose();
            goalPositions.Dispose();
            goalRadii.Dispose();
            goalPriorities.Dispose();
            tempGoalDist.Dispose();

            // 更新版本
            version++;
            sdfVersion++;
            directionVersion++;
        }

        /// <summary>
        /// 获取流场网格
        /// </summary>
        public FlowFieldGrid GetGrid() => grid;

        /// <summary>
        /// 获取指定位置的方向
        /// </summary>
        public float2 GetDirection(float2 worldPos)
        {
            int2 cell = grid.WorldToCellSafe(worldPos);
            if (!grid.IsValidCell(cell)) return float2.zero;
            return grid.Directions[grid.CellToIndex(cell)];
        }

        public int GetVersion() => version;
    }

    // ==================== Jobs ====================

    [BurstCompile]
    struct MarkObstaclesGridJob : IJob
    {
        [ReadOnly] public NativeArray<float2> ObstaclePositions;
        [ReadOnly] public NativeArray<float> ObstacleRadii;
        [ReadOnly] public NativeArray<byte> ObstacleShapes;
        [ReadOnly] public NativeArray<float2> ObstacleHalfExtents;

        [WriteOnly] public NativeArray<byte> CellTypes;
        [WriteOnly] public NativeArray<float> Distances;

        public int2 GridSize;
        public float CellSize;
        public float2 Origin;
        public float MaxDistance;

        public void Execute()
        {
            int total = GridSize.x * GridSize.y;

            for (int idx = 0; idx < total; idx++)
            {
                int x = idx % GridSize.x;
                int y = idx / GridSize.x;

                float2 cellCenter = Origin + new float2(x + 0.5f, y + 0.5f) * CellSize;

                float minSDF = MaxDistance;
                bool isObstacle = false;

                for (int i = 0; i < ObstaclePositions.Length; i++)
                {
                    float sdf;
                    if (ObstacleShapes[i] == 0)
                    {
                        sdf = FlowFieldMath.SDFCircle(cellCenter, ObstaclePositions[i], ObstacleRadii[i]);
                    }
                    else
                    {
                        sdf = FlowFieldMath.SDFBox(cellCenter, ObstaclePositions[i], ObstacleHalfExtents[i]);
                    }

                    if (sdf < 0f)
                    {
                        isObstacle = true;
                        break;
                    }

                    minSDF = math.min(minSDF, sdf);
                }

                if (isObstacle)
                {
                    CellTypes[idx] = (byte)CellType.Obstacle;
                    Distances[idx] = 0f;
                }
                else
                {
                    CellTypes[idx] = (byte)CellType.Passable;
                    Distances[idx] = minSDF / CellSize;
                }
            }
        }
    }

    [BurstCompile]
    struct MarkGoalsGridJob : IJob
    {
        [ReadOnly] public NativeArray<float2> GoalPositions;
        [ReadOnly] public NativeArray<float> GoalRadii;
        [ReadOnly] public NativeArray<int> GoalPriorities;

        [WriteOnly] public NativeArray<byte> CellTypes;
        [WriteOnly] public NativeArray<float> GoalDistances;

        public int2 GridSize;
        public float CellSize;
        public float2 Origin;

        public void Execute()
        {
            int total = GridSize.x * GridSize.y;

            for (int idx = 0; idx < total; idx++)
            {
                int x = idx % GridSize.x;
                int y = idx / GridSize.x;

                float2 cellCenter = Origin + new float2(x + 0.5f, y + 0.5f) * CellSize;

                bool hasGoal = false;
                float minScore = float.MaxValue;

                for (int i = 0; i < GoalPositions.Length; i++)
                {
                    float dist = math.length(cellCenter - GoalPositions[i]);

                    if (dist <= GoalRadii[i])
                    {
                        CellTypes[idx] = (byte)CellType.Goal;
                        GoalDistances[idx] = 0f;
                        hasGoal = true;
                        break;
                    }

                    float score = dist / math.max(1, GoalPriorities[i]);
                    if (score < minScore)
                    {
                        minScore = score;
                    }
                }

                if (!hasGoal)
                {
                    GoalDistances[idx] = minScore;
                }
            }
        }
    }

    [BurstCompile]
    struct InitGoalDistancesJob : IJob
    {
        [WriteOnly] public NativeArray<float> GoalDistances;
        [WriteOnly] public NativeArray<int> Integrated;

        public int TotalCells;
        public float InitValue;

        public void Execute()
        {
            for (int i = 0; i < TotalCells; i++)
            {
                GoalDistances[i] = InitValue;
                Integrated[i] = -1;
            }
        }
    }

    [BurstCompile]
    struct PropagateGoalDistancesJob : IJob
    {
        [ReadOnly] public NativeArray<float> InputDistances;
        [WriteOnly] public NativeArray<float> OutputDistances;
        [ReadOnly] public NativeArray<byte> CellTypes;

        public NativeArray<int> Integrated;

        public int2 GridSize;
        public int PassIndex;

        public void Execute()
        {
            int total = GridSize.x * GridSize.y;

            for (int idx = 0; idx < total; idx++)
            {
                if (CellTypes[idx] == (byte)CellType.Goal)
                {
                    OutputDistances[idx] = 0f;
                    Integrated[idx] = PassIndex;
                    continue;
                }

                if (CellTypes[idx] == (byte)CellType.Obstacle)
                {
                    OutputDistances[idx] = float.MaxValue;
                    continue;
                }

                int x = idx % GridSize.x;
                int y = idx / GridSize.x;

                float bestDist = InputDistances[idx];

                // 4方向邻居
                if (x > 0)
                {
                    int leftIdx = idx - 1;
                    if (CellTypes[leftIdx] != (byte)CellType.Obstacle)
                    {
                        float dist = InputDistances[leftIdx] + 1f;
                        if (dist < bestDist) bestDist = dist;
                    }
                }

                if (x < GridSize.x - 1)
                {
                    int rightIdx = idx + 1;
                    if (CellTypes[rightIdx] != (byte)CellType.Obstacle)
                    {
                        float dist = InputDistances[rightIdx] + 1f;
                        if (dist < bestDist) bestDist = dist;
                    }
                }

                if (y > 0)
                {
                    int downIdx = idx - GridSize.x;
                    if (CellTypes[downIdx] != (byte)CellType.Obstacle)
                    {
                        float dist = InputDistances[downIdx] + 1f;
                        if (dist < bestDist) bestDist = dist;
                    }
                }

                if (y < GridSize.y - 1)
                {
                    int upIdx = idx + GridSize.x;
                    if (CellTypes[upIdx] != (byte)CellType.Obstacle)
                    {
                        float dist = InputDistances[upIdx] + 1f;
                        if (dist < bestDist) bestDist = dist;
                    }
                }

                OutputDistances[idx] = bestDist;
            }
        }
    }

    [BurstCompile]
    struct ComputeFlowDirectionsJob : IJob
    {
        [ReadOnly] public NativeArray<float> GoalDistances;
        [ReadOnly] public NativeArray<byte> CellTypes;

        [WriteOnly] public NativeArray<float2> Directions;

        public int2 GridSize;

        public void Execute()
        {
            int total = GridSize.x * GridSize.y;

            for (int idx = 0; idx < total; idx++)
            {
                if (CellTypes[idx] == (byte)CellType.Obstacle)
                {
                    Directions[idx] = float2.zero;
                    continue;
                }

                int x = idx % GridSize.x;
                int y = idx / GridSize.x;

                float dist = GoalDistances[idx];
                float bestDx = 0f, bestDy = 0f;
                float bestDist = dist;

                // 检查4个邻居
                if (x > 0)
                {
                    int leftIdx = idx - 1;
                    if (CellTypes[leftIdx] != (byte)CellType.Obstacle)
                    {
                        if (GoalDistances[leftIdx] < bestDist)
                        {
                            bestDist = GoalDistances[leftIdx];
                            bestDx = -1f;
                            bestDy = 0f;
                        }
                    }
                }

                if (x < GridSize.x - 1)
                {
                    int rightIdx = idx + 1;
                    if (CellTypes[rightIdx] != (byte)CellType.Obstacle)
                    {
                        if (GoalDistances[rightIdx] < bestDist)
                        {
                            bestDist = GoalDistances[rightIdx];
                            bestDx = 1f;
                            bestDy = 0f;
                        }
                    }
                }

                if (y > 0)
                {
                    int downIdx = idx - GridSize.x;
                    if (CellTypes[downIdx] != (byte)CellType.Obstacle)
                    {
                        if (GoalDistances[downIdx] < bestDist)
                        {
                            bestDist = GoalDistances[downIdx];
                            bestDx = 0f;
                            bestDy = -1f;
                        }
                    }
                }

                if (y < GridSize.y - 1)
                {
                    int upIdx = idx + GridSize.x;
                    if (CellTypes[upIdx] != (byte)CellType.Obstacle)
                    {
                        if (GoalDistances[upIdx] < bestDist)
                        {
                            bestDist = GoalDistances[upIdx];
                            bestDx = 0f;
                            bestDy = 1f;
                        }
                    }
                }

                float2 dir = new float2(bestDx, bestDy);
                float lenSq = math.lengthsq(dir);

                if (lenSq < 1e-10f)
                {
                    Directions[idx] = float2.zero;
                }
                else
                {
                    Directions[idx] = dir / math.sqrt(lenSq);
                }
            }
        }
    }
}