using Unity.Entities;
using Unity.Mathematics;

namespace FlowField
{
    /// <summary>
    /// 流场代理组件 - 附加到每个需要寻路的实体
    /// </summary>
    public struct FlowFieldAgent : IComponentData
    {
        // ==================== 移动参数 ====================
        public float MoveSpeed;
        public float TurnSpeed;
        public float2 Velocity;
        public float2 Position;

        // ==================== 寻路参数 ====================
        public Entity CurrentGoal;
        public float ArrivalThreshold;
        public bool UseFlowField;

        // ==================== 状态 ====================
        public AgentState State;
        public int PathVersion;
        public float RemainingPathDistance;

        /// <summary>
        /// 默认配置
        /// </summary>
        public static FlowFieldAgent Default => new FlowFieldAgent
        {
            MoveSpeed = 5f,
            TurnSpeed = 10f,
            Velocity = float2.zero,
            Position = float2.zero,
            CurrentGoal = Entity.Null,
            ArrivalThreshold = 0.5f,
            UseFlowField = true,
            State = AgentState.Idle,
            PathVersion = -1,
            RemainingPathDistance = float.MaxValue
        };
    }

    /// <summary>
    /// 代理视野组件 - 定义代理的感知范围
    /// </summary>
    public struct FlowFieldAgentVision : IComponentData
    {
        public float ViewRadius;
        public float SeparationRadius;
        public float AlignmentRadius;
        public float CohesionRadius;

        public static FlowFieldAgentVision Default => new FlowFieldAgentVision
        {
            ViewRadius = 5f,
            SeparationRadius = 1f,
            AlignmentRadius = 3f,
            CohesionRadius = 5f
        };
    }

    /// <summary>
    /// 代理跟随组件 - 使代理跟随另一个实体
    /// </summary>
    public struct FlowFieldAgentFollow : IComponentData
    {
        public Entity Target;
        public float FollowDistance;
        public float ArrivalThreshold;

        public static FlowFieldAgentFollow Default => new FlowFieldAgentFollow
        {
            Target = Entity.Null,
            FollowDistance = 2f,
            ArrivalThreshold = 0.5f
        };
    }
}