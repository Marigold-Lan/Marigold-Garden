using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace FlowField.Jobs
{
    /// <summary>
    /// 空间哈希重建Job - 将所有代理和障碍物插入空间哈希表
    /// </summary>
    [BurstCompile(OptimizeFor = OptimizeFor.Throughput)]
    public struct SpatialHashRebuildJob : IJob
    {
        [ReadOnly] public NativeArray<float2> AgentPositions;
        [ReadOnly] public int AgentCount;

        [WriteOnly] public NativeArray<int> CellEntities;
        [WriteOnly] public NativeArray<int> CellCounts;
        [WriteOnly] public NativeHashMap<int, int2> EntityPosition;

        public float CellSize;
        public float InvCellSize;
        public int2 GridSize;
        public int CapacityPerCell;
        public float2 Origin;

        public void Execute()
        {
            // 清空所有格子
            CellCounts.AsSpan().Clear();
            EntityPosition.Clear();

            // 遍历所有代理并插入
            for (int i = 0; i < AgentCount; i++)
            {
                InsertAgent(i, AgentPositions[i]);
            }
        }

        private void InsertAgent(int agentId, float2 worldPos)
        {
            int2 cell = WorldToCellSafe(worldPos);
            int cellIdx = CellToIndex(cell);

            int count = CellCounts[cellIdx];
            if (count >= CapacityPerCell) return;

            int offset = cellIdx * CapacityPerCell;
            CellEntities[offset + count] = agentId;
            CellCounts[cellIdx] = count + 1;

            if (!EntityPosition.ContainsKey(agentId))
            {
                EntityPosition.Add(agentId, cell);
            }
        }

        [GenerateTestsForBurstCompatibility]
        private int2 WorldToCellSafe(float2 worldPos)
        {
            int2 cell = new int2(
                (int)math.floor((worldPos.x - Origin.x) * InvCellSize),
                (int)math.floor((worldPos.y - Origin.y) * InvCellSize)
            );
            cell.x = ((cell.x % GridSize.x) + GridSize.x) % GridSize.x;
            cell.y = ((cell.y % GridSize.y) + GridSize.y) % GridSize.y;
            return cell;
        }

        [GenerateTestsForBurstCompatibility]
        private int CellToIndex(int2 cell)
        {
            return cell.y * GridSize.x + cell.x;
        }
    }

    /// <summary>
    /// 空间哈希增量更新Job - 只更新移动的实体
    /// </summary>
    [BurstCompile(OptimizeFor = OptimizeFor.Throughput)]
    public struct SpatialHashUpdateJob : IJob
    {
        [ReadOnly] public NativeArray<int> UpdatedEntityIds;
        [ReadOnly] public int UpdateCount;
        [ReadOnly] public NativeArray<float2> UpdatedPositions;

        [WriteOnly] public NativeArray<int> CellEntities;
        [WriteOnly] public NativeArray<int> CellCounts;
        public NativeHashMap<int, int2> EntityPosition;

        public float CellSize;
        public float InvCellSize;
        public int2 GridSize;
        public int CapacityPerCell;
        public float2 Origin;

        public void Execute()
        {
            for (int i = 0; i < UpdateCount; i++)
            {
                int entityId = UpdatedEntityIds[i];
                float2 newPos = UpdatedPositions[i];

                if (EntityPosition.TryGetValue(entityId, out int2 oldCell))
                {
                    int2 newCell = WorldToCellSafe(newPos);

                    if (oldCell.x != newCell.x || oldCell.y != newCell.y)
                    {
                        RemoveFromCell(entityId, oldCell);
                        InsertToCell(entityId, newPos, newCell);
                        EntityPosition[entityId] = newCell;
                    }
                }
                else
                {
                    InsertAgent(entityId, newPos);
                }
            }
        }

        private void InsertAgent(int agentId, float2 worldPos)
        {
            int2 cell = WorldToCellSafe(worldPos);
            InsertToCell(agentId, worldPos, cell);
        }

        private void InsertToCell(int agentId, float2 worldPos, int2 cell)
        {
            int cellIdx = CellToIndex(cell);
            int count = CellCounts[cellIdx];
            if (count >= CapacityPerCell) return;

            int offset = cellIdx * CapacityPerCell;
            CellEntities[offset + count] = agentId;
            CellCounts[cellIdx] = count + 1;

            if (!EntityPosition.ContainsKey(agentId))
            {
                EntityPosition.Add(agentId, cell);
            }
            else
            {
                EntityPosition[agentId] = cell;
            }
        }

        private void RemoveFromCell(int agentId, int2 cell)
        {
            int cellIdx = CellToIndex(cell);
            int count = CellCounts[cellIdx];
            int offset = cellIdx * CapacityPerCell;

            for (int i = 0; i < count; i++)
            {
                if (CellEntities[offset + i] == agentId)
                {
                    for (int j = i; j < count - 1; j++)
                    {
                        CellEntities[offset + j] = CellEntities[offset + j + 1];
                    }
                    CellCounts[cellIdx] = count - 1;
                    return;
                }
            }
        }

        [GenerateTestsForBurstCompatibility]
        private int2 WorldToCellSafe(float2 worldPos)
        {
            int2 cell = new int2(
                (int)math.floor((worldPos.x - Origin.x) * InvCellSize),
                (int)math.floor((worldPos.y - Origin.y) * InvCellSize)
            );
            cell.x = ((cell.x % GridSize.x) + GridSize.x) % GridSize.x;
            cell.y = ((cell.y % GridSize.y) + GridSize.y) % GridSize.y;
            return cell;
        }

        [GenerateTestsForBurstCompatibility]
        private int CellToIndex(int2 cell)
        {
            return cell.y * GridSize.x + cell.x;
        }
    }

    /// <summary>
    /// 邻居查询Job - 批量查询某位置周围的实体
    /// </summary>
    [BurstCompile(OptimizeFor = OptimizeFor.Throughput)]
    public struct NeighborQueryJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float2> QueryPositions;
        [ReadOnly] public float QueryRadius;

        [ReadOnly] public NativeArray<int> CellEntities;
        [ReadOnly] public NativeArray<int> CellCounts;
        [ReadOnly] public int CapacityPerCell;
        [ReadOnly] public int2 GridSize;

        public NativeArray<int> ResultCounts;
        [WriteOnly] public NativeArray<int> ResultEntities;

        public float CellSize;
        public float InvCellSize;
        public float2 Origin;

        public void Execute(int index)
        {
            float2 center = QueryPositions[index];
            int2 minCell = WorldToCellSafe(center - QueryRadius);
            int2 maxCell = WorldToCellSafe(center + QueryRadius);

            int resultOffset = index * CapacityPerCell;
            int count = 0;

            for (int y = minCell.y; y <= maxCell.y; y++)
            {
                for (int x = minCell.x; x <= maxCell.x; x++)
                {
                    int2 cell = new int2(x, y);
                    int cellIdx = CellToIndex(cell);
                    int cellCount = CellCounts[cellIdx];

                    if (cellCount == 0) continue;

                    int offset = cellIdx * CapacityPerCell;
                    for (int i = 0; i < cellCount; i++)
                    {
                        int entityId = CellEntities[offset + i];
                        ResultEntities[resultOffset + count] = entityId;
                        count++;
                    }
                }
            }

            ResultCounts[index] = count;
        }

        [GenerateTestsForBurstCompatibility]
        private int2 WorldToCellSafe(float2 worldPos)
        {
            int2 cell = new int2(
                (int)math.floor((worldPos.x - Origin.x) * InvCellSize),
                (int)math.floor((worldPos.y - Origin.y) * InvCellSize)
            );
            cell.x = ((cell.x % GridSize.x) + GridSize.x) % GridSize.x;
            cell.y = ((cell.y % GridSize.y) + GridSize.y) % GridSize.y;
            return cell;
        }

        [GenerateTestsForBurstCompatibility]
        private int CellToIndex(int2 cell)
        {
            return cell.y * GridSize.x + cell.x;
        }
    }
}