using Unity.Collections;
using Unity.Mathematics;
using Unity.Burst;

namespace FlowField
{
    /// <summary>
    /// 流场网格核心数据结构
    /// 采用SoA (Structure of Arrays) 布局优化内存访问
    /// </summary>
    public struct FlowFieldGrid : IDisposable
    {
        // ==================== 配置 ====================
        public readonly int2 GridSize;
        public readonly float CellSize;
        public readonly float2 Origin;

        // ==================== SoA布局数据 ====================
        public NativeArray<float> Distances;
        public NativeArray<float2> Directions;
        public NativeArray<float> GoalDistances;
        public NativeArray<byte> CellTypes;
        public NativeArray<int> Integrated;
        public NativeArray<int> GoalIntegrated;

        // ==================== 状态 ====================
        public int Version;
        public int SDFVersion;
        public int GoalVersion;

        // ==================== 工具属性 ====================
        public int TotalCells => GridSize.x * GridSize.y;
        public float WorldWidth => GridSize.x * CellSize;
        public float WorldHeight => GridSize.y * CellSize;
        public float2 BoundsMin => Origin;
        public float2 BoundsMax => Origin + new float2(GridSize.x * CellSize, GridSize.y * CellSize);

        /// <summary>
        /// 创建新的流场网格
        /// </summary>
        public static FlowFieldGrid Create(int2 gridSize, float cellSize, float2 origin, Allocator allocator)
        {
            int total = gridSize.x * gridSize.y;
            return new FlowFieldGrid
            {
                GridSize = gridSize,
                CellSize = cellSize,
                Origin = origin,
                Distances = new NativeArray<float>(total, allocator),
                Directions = new NativeArray<float2>(total, allocator),
                GoalDistances = new NativeArray<float>(total, allocator),
                CellTypes = new NativeArray<byte>(total, allocator),
                Integrated = new NativeArray<int>(total, allocator),
                GoalIntegrated = new NativeArray<int>(total, allocator),
                Version = 0,
                SDFVersion = 0,
                GoalVersion = 0
            };
        }

        /// <summary>
        /// 世界坐标转网格坐标
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public int2 WorldToCell(float2 worldPos)
        {
            int2 cell = (int2)math.floor((worldPos - Origin) / CellSize);
            return cell;
        }

        /// <summary>
        /// 世界坐标转网格坐标（安全版本，超出边界会裁剪）
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public int2 WorldToCellSafe(float2 worldPos)
        {
            int2 cell = (int2)math.floor((worldPos - Origin) / CellSize);
            cell.x = math.clamp(cell.x, 0, GridSize.x - 1);
            cell.y = math.clamp(cell.y, 0, GridSize.y - 1);
            return cell;
        }

        /// <summary>
        /// 网格坐标转世界坐标（单元格中心）
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public float2 CellToWorld(int2 cell)
        {
            return Origin + (cell + new float2(0.5f, 0.5f)) * CellSize;
        }

        /// <summary>
        /// 网格坐标转世界坐标（指定单元格角点）
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public float2 CellToWorldCorner(int2 cell)
        {
            return Origin + cell * CellSize;
        }

        /// <summary>
        /// 索引转换
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public int CellToIndex(int2 cell)
        {
            return cell.y * GridSize.x + cell.x;
        }

        /// <summary>
        /// 索引转换（安全版本）
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public int CellToIndexSafe(int2 cell)
        {
            if (cell.x < 0 || cell.x >= GridSize.x || cell.y < 0 || cell.y >= GridSize.y)
                return -1;
            return cell.y * GridSize.x + cell.x;
        }

        /// <summary>
        /// 从索引获取单元格坐标
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public int2 IndexToCell(int index)
        {
            return new int2(index % GridSize.x, index / GridSize.x);
        }

        /// <summary>
        /// 安全检查
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public bool IsValidCell(int2 cell)
        {
            return (uint)cell.x < (uint)GridSize.x && (uint)cell.y < (uint)GridSize.y;
        }

        /// <summary>
        /// 获取单元格世界边界
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public float4 GetCellBounds(int2 cell)
        {
            float2 min = Origin + cell * CellSize;
            return new float4(min, min + CellSize);
        }

        /// <summary>
        /// 检查世界坐标是否在网格内
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public bool IsInBounds(float2 worldPos)
        {
            return worldPos.x >= Origin.x && worldPos.x < Origin.x + GridSize.x * CellSize &&
                   worldPos.y >= Origin.y && worldPos.y < Origin.y + GridSize.y * CellSize;
        }

        /// <summary>
        /// 获取单元格对应的流场方向
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public float2 GetDirection(int2 cell)
        {
            int idx = CellToIndexSafe(cell);
            if (idx < 0) return new float2(0, 0);
            return Directions[idx];
        }

        /// <summary>
        /// 获取单元格对应的Goal距离
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public float GetGoalDistance(int2 cell)
        {
            int idx = CellToIndexSafe(cell);
            if (idx < 0) return float.MaxValue;
            return GoalDistances[idx];
        }

        /// <summary>
        /// 双线性插值采样方向场
        /// </summary>
        [GenerateTestsForBurstCompatibility]
        public float2 SampleDirectionBilinear(float2 worldPos)
        {
            float2 normalizedPos = (worldPos - Origin) / CellSize;
            float2 cellPos = normalizedPos - 0.5f;

            int2 cell0 = new int2(math.floor(cellPos.x), math.floor(cellPos.y));
            float2 frac = cellPos - new float2(cell0);

            int2 cell00 = cell0;
            int2 cell10 = cell0 + new int2(1, 0);
            int2 cell01 = cell0 + new int2(0, 1);
            int2 cell11 = cell0 + new int2(1, 1);

            float2 dir00 = IsValidCell(cell00) ? Directions[CellToIndex(cell00)] : float2.zero;
            float2 dir10 = IsValidCell(cell10) ? Directions[CellToIndex(cell10)] : float2.zero;
            float2 dir01 = IsValidCell(cell01) ? Directions[CellToIndex(cell01)] : float2.zero;
            float2 dir11 = IsValidCell(cell11) ? Directions[CellToIndex(cell11)] : float2.zero;

            float2 dirX0 = math.lerp(dir00, dir10, frac.x);
            float2 dirX1 = math.lerp(dir01, dir11, frac.x);
            return math.normalize(dirX0 + dirX1 * frac.y);
        }

        /// <summary>
        /// 清空网格数据
        /// </summary>
        public void Clear()
        {
            Distances.AsSpan().Fill(float.MaxValue);
            Directions.AsSpan().Fill(float2.zero);
            GoalDistances.AsSpan().Fill(float.MaxValue);
            CellTypes.AsSpan().Fill((byte)CellType.Passable);
            Integrated.AsSpan().Fill(-1);
            GoalIntegrated.AsSpan().Fill(-1);
        }

        public void Dispose()
        {
            if (Distances.IsCreated)
            {
                Distances.Dispose();
                Directions.Dispose();
                GoalDistances.Dispose();
                CellTypes.Dispose();
                Integrated.Dispose();
                GoalIntegrated.Dispose();
            }
        }
    }
}