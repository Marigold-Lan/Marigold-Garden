using Unity.Entities;
using Unity.Mathematics;

namespace FlowField
{
    /// <summary>
    /// 流场网格数据组件 - 每个网格区域一个
    /// 存储网格配置和状态信息
    /// </summary>
    public struct FlowFieldGridData : IComponentData
    {
        // ==================== 网格配置 ====================
        public int2 GridSize;
        public float CellSize;
        public float2 Origin;
        public float2 BoundsMin;
        public float2 BoundsMax;

        // ==================== 更新控制 ====================
        public int UpdateInterval;
        public int LastUpdateFrame;
        public bool NeedsFullRebuild;
        public bool IsStatic;

        // ==================== 版本控制 ====================
        public int SDFVersion;
        public int DirectionVersion;
        public int GoalVersion;
        public int Version;

        /// <summary>
        /// 默认配置
        /// </summary>
        public static FlowFieldGridData Default => new FlowFieldGridData
        {
            GridSize = new int2(100, 100),
            CellSize = 1f,
            Origin = new float2(-50f, -50f),
            BoundsMin = new float2(-50f, -50f),
            BoundsMax = new float2(50f, 50f),
            UpdateInterval = 1,
            LastUpdateFrame = -1,
            NeedsFullRebuild = true,
            IsStatic = false,
            SDFVersion = 0,
            DirectionVersion = 0,
            GoalVersion = 0,
            Version = 0
        };

        /// <summary>
        /// 创建新配置
        /// </summary>
        public static FlowFieldGridData Create(float2 worldBoundsMin, float2 worldBoundsMax, float cellSize, int updateInterval = 1)
        {
            float2 size = worldBoundsMax - worldBoundsMin;
            int2 gridSize = new int2(
                math.ceil(size.x / cellSize),
                math.ceil(size.y / cellSize)
            );

            return new FlowFieldGridData
            {
                GridSize = gridSize,
                CellSize = cellSize,
                Origin = worldBoundsMin,
                BoundsMin = worldBoundsMin,
                BoundsMax = worldBoundsMax,
                UpdateInterval = updateInterval,
                LastUpdateFrame = -1,
                NeedsFullRebuild = true,
                IsStatic = false,
                SDFVersion = 0,
                DirectionVersion = 0,
                GoalVersion = 0,
                Version = 0
            };
        }

        /// <summary>
        /// 获取总单元格数
        /// </summary>
        public int TotalCells => GridSize.x * GridSize.y;

        /// <summary>
        /// 检查是否需要更新
        /// </summary>
        public bool NeedsUpdate(int currentFrame)
        {
            if (NeedsFullRebuild) return true;
            return (currentFrame - LastUpdateFrame) >= UpdateInterval;
        }

        /// <summary>
        /// 标记为需要重建
        /// </summary>
        public void MarkDirty()
        {
            NeedsFullRebuild = true;
            Version++;
        }

        /// <summary>
        /// 标记为增量更新
        /// </summary>
        public void MarkIncrementalDirty()
        {
            NeedsFullRebuild = false;
            Version++;
        }
    }

    /// <summary>
    /// 流场引用组件 - 代理引用流场数据的指针
    /// </summary>
    public struct FlowFieldGridRef : IComponentData
    {
        public Entity GridEntity;
    }

    /// <summary>
    /// 流场更新请求组件 - 用于请求更新特定区域
    /// </summary>
    public struct FlowFieldUpdateRequest : IComponentData
    {
        public float2 Center;
        public float Radius;
        public UpdatePriority Priority;

        public static FlowFieldUpdateRequest Create(float2 center, float radius, UpdatePriority priority = UpdatePriority.Normal)
        {
            return new FlowFieldUpdateRequest
            {
                Center = center,
                Radius = radius,
                Priority = priority
            };
        }
    }

    public enum UpdatePriority : byte
    {
        Low = 0,
        Normal = 1,
        High = 2,
        Critical = 3
    }
}