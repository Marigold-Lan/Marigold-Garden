using Unity.Mathematics;

namespace FlowField
{
    /// <summary>
    /// 流场配置数据 - 定义流场网格的基本参数
    /// </summary>
    public struct FlowFieldConfig
    {
        /// <summary>
        /// 网格单元格大小（世界单位）
        /// 建议值为代理半径的2-4倍
        /// </summary>
        public float CellSize;

        /// <summary>
        /// 网格世界空间边界
        /// </summary>
        public float2 WorldBoundsMin;
        public float2 WorldBoundsMax;

        /// <summary>
        /// 代理半径（用于碰撞检测）
        /// </summary>
        public float AgentRadius;

        /// <summary>
        /// 代理移动速度
        /// </summary>
        public float AgentSpeed;

        /// <summary>
        /// 转向速度（弧度/秒）
        /// </summary>
        public float TurnSpeed;

        /// <summary>
        /// 到达阈值距离
        /// </summary>
        public float ArrivalThreshold;

        /// <summary>
        /// 邻居排斥力权重
        /// </summary>
        public float SeparationWeight;

        /// <summary>
        /// 最大邻居数量
        /// </summary>
        public int MaxNeighbors;

        /// <summary>
        /// 流场更新间隔（帧数）
        /// </summary>
        public int UpdateInterval;

        /// <summary>
        /// SDF计算的最大迭代次数（JFA算法）
        /// </summary>
        public int MaxSDFIterations;

        /// <summary>
        /// 默认配置
        /// </summary>
        public static FlowFieldConfig Default => new FlowFieldConfig
        {
            CellSize = 1.0f,
            WorldBoundsMin = new float2(-50f, -50f),
            WorldBoundsMax = new float2(50f, 50f),
            AgentRadius = 0.25f,
            AgentSpeed = 5f,
            TurnSpeed = 10f,
            ArrivalThreshold = 0.5f,
            SeparationWeight = 1.5f,
            MaxNeighbors = 16,
            UpdateInterval = 1,
            MaxSDFIterations = 8
        };

        /// <summary>
        /// 从世界范围计算网格尺寸
        /// </summary>
        public int2 CalculateGridSize()
        {
            float2 size = WorldBoundsMax - WorldBoundsMin;
            return new int2(
                (int)math.ceil(size.x / CellSize),
                (int)math.ceil(size.y / CellSize)
            );
        }
    }

    /// <summary>
    /// 单元格类型枚举
    /// </summary>
    public enum CellType : byte
    {
        Passable = 0,      // 可通行区域
        Obstacle = 1,      // 障碍物
        Goal = 2           // 目标点
    }

    /// <summary>
    /// 代理寻路状态
    /// </summary>
    public enum AgentState : byte
    {
        Idle = 0,
        Seeking = 1,
        Arrived = 2,
        Blocked = 3,
        NoPath = 4
    }

    /// <summary>
    /// 障碍物形状类型
    /// </summary>
    public enum ObstacleShape : byte
    {
        Circle = 0,
        Box = 1
    }
}