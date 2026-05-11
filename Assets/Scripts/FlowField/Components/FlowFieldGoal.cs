using Unity.Entities;
using Unity.Mathematics;

namespace FlowField
{
    /// <summary>
    /// 目标点组件 - 定义流场的汇点
    /// 支持多目标点竞争场景
    /// </summary>
    public struct FlowFieldGoal : IComponentData
    {
        // ==================== 位置 ====================
        public float2 Position;

        // ==================== 影响范围 ====================
        public float Radius;
        public float InfluenceRadius;

        // ==================== 优先级 ====================
        public int Priority;
        public float Weight;

        // ==================== 状态 ====================
        public bool IsActive;
        public int GoalIndex;

        /// <summary>
        /// 默认配置
        /// </summary>
        public static FlowFieldGoal Default => new FlowFieldGoal
        {
            Position = float2.zero,
            Radius = 0.5f,
            InfluenceRadius = 5f,
            Priority = 1,
            Weight = 1f,
            IsActive = true,
            GoalIndex = -1
        };

        /// <summary>
        /// 创建点目标
        /// </summary>
        public static FlowFieldGoal CreatePoint(float2 position, int priority = 1)
        {
            return new FlowFieldGoal
            {
                Position = position,
                Radius = 0.5f,
                InfluenceRadius = 5f,
                Priority = priority,
                Weight = 1f,
                IsActive = true,
                GoalIndex = -1
            };
        }

        /// <summary>
        /// 创建区域目标
        /// </summary>
        public static FlowFieldGoal CreateArea(float2 position, float radius, int priority = 1)
        {
            return new FlowFieldGoal
            {
                Position = position,
                Radius = radius,
                InfluenceRadius = radius * 2f,
                Priority = priority,
                Weight = 1f,
                IsActive = true,
                GoalIndex = -1
            };
        }

        /// <summary>
        /// 计算代理到目标的评分（距离/优先级）
        /// </summary>
        public float CalculateScore(float2 agentPosition)
        {
            float distance = FlowFieldMath.Distance(agentPosition, Position);
            return distance / math.max(1, Priority);
        }

        /// <summary>
        /// 检查目标是否影响指定位置
        /// </summary>
        public bool IsInInfluence(float2 position)
        {
            return FlowFieldMath.Distance(position, Position) <= InfluenceRadius;
        }

        /// <summary>
        /// 检查代理是否到达目标
        /// </summary>
        public bool IsReached(float2 agentPosition)
        {
            return FlowFieldMath.Distance(agentPosition, Position) <= Radius;
        }
    }

    /// <summary>
    /// 目标组组件 - 用于管理多个相关目标
    /// </summary>
    public struct FlowFieldGoalGroup : IComponentData
    {
        public int GoalCount;
        public Entity FirstGoal;
        public Entity SecondGoal;
        public Entity ThirdGoal;
        public Entity FourthGoal;
    }

    /// <summary>
    /// 目标选择组件 - 决定代理如何选择目标
    /// </summary>
    public struct FlowFieldGoalSelector : IComponentData
    {
        public GoalSelectionMode Mode;

        public static FlowFieldGoalSelector Default => new FlowFieldGoalSelector
        {
            Mode = GoalSelectionMode.Nearest
        };
    }

    public enum GoalSelectionMode : byte
    {
        Nearest,          // 选择最近的目标
        HighestPriority,  // 选择优先级最高的目标
        WeightedRandom,   // 根据权重随机选择
        NearestPriority   // 选择（距离/优先级）最小的目标
    }
}