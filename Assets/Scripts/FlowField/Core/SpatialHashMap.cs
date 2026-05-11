using Unity.Collections;
using Unity.Mathematics;
using Unity.Burst;

namespace FlowField
{
    /// <summary>
    /// 空间哈希映射表
    /// 用于高效邻居查询，支持动态障碍物和化学交互
    /// 使用开放地址法哈希表实现
    /// </summary>
    public struct SpatialHashMap : IDisposable
    {
        // ==================== 配置 ====================
        public readonly float CellSize;
        public readonly float InvCellSize;
        public readonly int2 GridSize;
        public readonly int TotalCells;
        public readonly int Capacity;

        // ==================== 数据存储 ====================
        public NativeArray<int2> CellEntities;  // 每个空间格子存储的实体索引列表
        public NativeArray<int> CellCounts;     // 每个格子中的实体数量
        public NativeHashMap<int, int2> EntityPosition; // 实体索引 -> 格子坐标

        // ==================== 状态 ====================
        public int Version;
        public int EntityCount;

        /// <summary>
        /// 创建空间哈希表
        /// </summary>
        public static SpatialHashMap Create(float2 worldBoundsMin, float2 worldBoundsMax, float cellSize, int capacity, Allocator allocator)
        {
            float2 size = worldBoundsMax - worldBoundsMin;
            int2 gridSize = new int2(
                math.ceil(size.x / cellSize),
                math.ceil(size.y / cellSize)
            );
            int totalCells = gridSize.x * gridSize.y;

            int capacityPerCell = math.max(4, capacity / totalCells);

            return new SpatialHashMap
            {
                CellSize = cellSize,
                InvCellSize = 1f / cellSize,
                GridSize = gridSize,
                TotalCells = totalCells,
                Capacity = capacity,
                CellEntities = new NativeArray<int2>(totalCells * capacityPerCell, allocator),
                CellCounts = new NativeArray<int>(totalCells, allocator),
                EntityPosition = new NativeHashMap<int, int2>(capacity, allocator),
                Version = 0,
                EntityCount = 0
            };
        }

        /// <summary>
        /// 世界坐标转格子坐标
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public int2 WorldToCell(float2 worldPos, float2 origin)
        {
            return (int2)math.floor((worldPos - origin) * InvCellSize);
        }

        /// <summary>
        /// 格子坐标转索引
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public int CellToIndex(int2 cell)
        {
            int x = ((cell.x % GridSize.x) + GridSize.x) % GridSize.x;
            int y = ((cell.y % GridSize.y) + GridSize.y) % GridSize.y;
            return y * GridSize.x + x;
        }

        /// <summary>
        /// 安全获取格子坐标（带边界检查）
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public int2 WorldToCellSafe(float2 worldPos, float2 origin)
        {
            int2 cell = WorldToCell(worldPos, origin);
            cell.x = ((cell.x % GridSize.x) + GridSize.x) % GridSize.x;
            cell.y = ((cell.y % GridSize.y) + GridSize.y) % GridSize.y;
            return cell;
        }

        /// <summary>
        /// 插入实体
        /// </summary>
        public void Insert(int entityId, float2 worldPos, float2 origin)
        {
            int2 cell = WorldToCellSafe(worldPos, origin);
            int cellIdx = CellToIndex(cell);

            int count = CellCounts[cellIdx];
            int offset = cellIdx * GetCapacityPerCell();

            CellEntities[offset + count] = new int2(entityId, 0);
            CellCounts[cellIdx] = count + 1;
            EntityPosition[entityId] = cell;
            EntityCount++;
            Version++;
        }

        /// <summary>
        /// 更新实体位置
        /// </summary>
        public bool Update(int entityId, float2 newWorldPos, float2 origin)
        {
            if (!EntityPosition.TryGetValue(entityId, out int2 oldCell))
                return false;

            int2 newCell = WorldToCellSafe(newWorldPos, origin);

            if (oldCell.x == newCell.x && oldCell.y == newCell.y)
                return false;

            int oldCellIdx = CellToIndex(oldCell);
            int newCellIdx = CellToIndex(newCell);

            RemoveFromCell(entityId, oldCellIdx);
            Insert(entityId, newWorldPos, origin);
            return true;
        }

        /// <summary>
        /// 移除实体
        /// </summary>
        public bool Remove(int entityId)
        {
            if (!EntityPosition.TryGetValue(entityId, out int2 cell))
                return false;

            int cellIdx = CellToIndex(cell);
            RemoveFromCell(entityId, cellIdx);
            EntityPosition.Remove(entityId);
            EntityCount--;
            Version++;
            return true;
        }

        private void RemoveFromCell(int entityId, int cellIdx)
        {
            int count = CellCounts[cellIdx];
            int capacityPerCell = GetCapacityPerCell();
            int offset = cellIdx * capacityPerCell;

            for (int i = 0; i < count; i++)
            {
                if (CellEntities[offset + i].x == entityId)
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

        /// <summary>
        /// 查询半径范围内的实体
        /// </summary>
        public void QueryRadius(float2 center, float radius, float2 origin, NativeList<int>.ParallelWriter results, int jobId)
        {
            int2 minCell = WorldToCellSafe(center - radius, origin);
            int2 maxCell = WorldToCellSafe(center + radius, origin);

            for (int y = minCell.y; y <= maxCell.y; y++)
            {
                for (int x = minCell.x; x <= maxCell.x; x++)
                {
                    int2 cell = new int2(x, y);
                    int cellIdx = CellToIndex(cell);
                    int count = CellCounts[cellIdx];

                    if (count == 0) continue;

                    int offset = cellIdx * GetCapacityPerCell();
                    for (int i = 0; i < count; i++)
                    {
                        results.AddNoResize(CellEntities[offset + i].x);
                    }
                }
            }
        }

        /// <summary>
        /// 同步查询（返回数组）
        /// </summary>
        public int QueryRadius(float2 center, float radius, float2 origin, NativeArray<int> results, int startIndex)
        {
            int2 minCell = WorldToCellSafe(center - radius, origin);
            int2 maxCell = WorldToCellSafe(center + radius, origin);

            int count = 0;
            for (int y = minCell.y; y <= maxCell.y; y++)
            {
                for (int x = minCell.x; x <= maxCell.x; x++)
                {
                    int2 cell = new int2(x, y);
                    int cellIdx = CellToIndex(cell);
                    int cellCount = CellCounts[cellIdx];

                    if (cellCount == 0) continue;

                    int offset = cellIdx * GetCapacityPerCell();
                    for (int i = 0; i < cellCount; i++)
                    {
                        results[startIndex + count] = CellEntities[offset + i].x;
                        count++;
                    }
                }
            }
            return count;
        }

        /// <summary>
        /// 清空所有实体
        /// </summary>
        public void Clear()
        {
            CellCounts.AsSpan().Clear();
            EntityPosition.Clear();
            EntityCount = 0;
            Version++;
        }

        /// <summary>
        /// 获取每个格子的容量
        /// </summary>
        private int GetCapacityPerCell()
        {
            return CellEntities.Length / math.max(1, TotalCells);
        }

        public void Dispose()
        {
            if (CellEntities.IsCreated)
            {
                CellEntities.Dispose();
                CellCounts.Dispose();
                EntityPosition.Dispose();
            }
        }
    }

    /// <summary>
    /// 空间哈希查询结果包装器
    /// </summary>
    public struct SpatialHashQuery : IDisposable
    {
        public NativeList<int> Entities;
        public NativeList<float2> Positions;
        public int Count;

        public static SpatialHashQuery Create(Allocator allocator)
        {
            return new SpatialHashQuery
            {
                Entities = new NativeList<int>(64, allocator),
                Positions = new NativeList<float2>(64, allocator),
                Count = 0
            };
        }

        public void Clear()
        {
            Entities.Clear();
            Positions.Clear();
            Count = 0;
        }

        public void Dispose()
        {
            if (Entities.IsCreated)
            {
                Entities.Dispose();
                Positions.Dispose();
            }
        }
    }
}