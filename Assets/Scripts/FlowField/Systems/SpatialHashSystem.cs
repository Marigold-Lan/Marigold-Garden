using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace FlowField.Systems
{
    /// <summary>
    /// 空间哈希系统 - 管理空间哈希表的更新
    /// </summary>
    public partial class SpatialHashSystem : SystemBase
    {
        private NativeParallelMultiHashMap<int2, int> spatialHash;
        private NativeHashMap<int, int2> entityToCell;
        private int version;

        private float cellSize;
        private float2 worldBoundsMin;
        private float2 worldBoundsMax;
        private int2 gridSize;

        protected override void OnCreate()
        {
            worldBoundsMin = new float2(-50f, -50f);
            worldBoundsMax = new float2(50f, 50f);
            cellSize = 1f;

            float2 size = worldBoundsMax - worldBoundsMin;
            gridSize = new int2(
                (int)math.ceil(size.x / cellSize),
                (int)math.ceil(size.y / cellSize)
            );

            int capacity = gridSize.x * gridSize.y;
            spatialHash = new NativeParallelMultiHashMap<int2, int>(capacity, Allocator.Persistent);
            entityToCell = new NativeHashMap<int, int2>(4096, Allocator.Persistent);
            version = 0;
        }

        protected override void OnDestroy()
        {
            if (spatialHash.IsCreated) spatialHash.Dispose();
            if (entityToCell.IsCreated) entityToCell.Dispose();
        }

        protected override void OnUpdate()
        {
            // 清空哈希表
            spatialHash.Clear();

            // 收集所有代理位置
            NativeArray<float2> agentPositions = new NativeArray<float2>(4096, Allocator.TempJob);
            NativeList<int> agentIds = new NativeList<int>(4096, Allocator.TempJob);

            foreach (var (agent, entity) in SystemAPI.Query<RefRO<FlowFieldAgent>>().WithEntityAccess())
            {
                int id = entity.Index;
                agentIds.Add(id);
                agentPositions[id] = agent.ValueRO.Position;
            }

            // 重建哈希表
            var rebuildJob = new RebuildSpatialHashJob
            {
                AgentIds = agentIds.AsArray(),
                AgentPositions = agentPositions,
                SpatialHash = spatialHash.AsParallelWriter(),
                EntityToCell = entityToCell,
                CellSize = cellSize,
                GridSize = gridSize,
                WorldBoundsMin = worldBoundsMin
            };

            JobHandle handle = rebuildJob.Schedule(agentIds.Length, 64);
            handle.Complete();

            agentPositions.Dispose();
            agentIds.Dispose();

            version++;
        }

        [BurstCompile]
        struct RebuildSpatialHashJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> AgentIds;
            [ReadOnly] public NativeArray<float2> AgentPositions;

            public NativeParallelMultiHashMap<int2, int>.ParallelWriter SpatialHash;
            public NativeHashMap<int, int2> EntityToCell;

            public float CellSize;
            public int2 GridSize;
            public float2 WorldBoundsMin;

            public void Execute(int index)
            {
                int agentId = AgentIds[index];
                float2 position = AgentPositions[index];

                int2 cell = WorldToCell(position);

                SpatialHash.Add(cell, agentId);
                EntityToCell[agentId] = cell;
            }

            private int2 WorldToCell(float2 worldPos)
            {
                return new int2(
                    (int)math.floor((worldPos.x - WorldBoundsMin.x) / CellSize),
                    (int)math.floor((worldPos.y - WorldBoundsMin.y) / CellSize)
                );
            }
        }

        /// <summary>
        /// 查询某位置的邻居
        /// </summary>
        public NativeList<int> QueryNeighbors(float2 position, float radius)
        {
            var results = new NativeList<int>(64, Allocator.Temp);

            int2 centerCell = WorldToCell(position);
            int radiusCells = (int)math.ceil(radius / cellSize);

            for (int dy = -radiusCells; dy <= radiusCells; dy++)
            {
                for (int dx = -radiusCells; dx <= radiusCells; dx++)
                {
                    int2 cell = centerCell + new int2(dx, dy);

                    if ((uint)cell.x >= (uint)gridSize.x || (uint)cell.y >= (uint)gridSize.y)
                        continue;

                    if (spatialHash.TryGetFirstValue(cell, out int entityId, out var iterator))
                    {
                        do
                        {
                            results.Add(entityId);
                        } while (spatialHash.TryGetNextValue(out entityId, ref iterator));
                    }
                }
            }

            return results;
        }

        private int2 WorldToCell(float2 worldPos)
        {
            return new int2(
                (int)math.floor((worldPos.x - worldBoundsMin.x) / cellSize),
                (int)math.floor((worldPos.y - worldBoundsMin.y) / cellSize)
            );
        }

        public int GetVersion() => version;
    }
}