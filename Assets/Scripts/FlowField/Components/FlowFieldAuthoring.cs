using Unity.Entities;
using Unity.Mathematics;

namespace FlowField
{
    /// <summary>
    /// 代理Authoring数据 - 用于烘焙到IComponentData
    /// </summary>
    public struct FlowFieldAgentAuthoring
    {
        public float MoveSpeed;
        public float TurnSpeed;
        public float ArrivalThreshold;
        public bool UseFlowField;
        public float ViewRadius;
        public float SeparationRadius;

        public static FlowFieldAgentAuthoring Default => new FlowFieldAgentAuthoring
        {
            MoveSpeed = 5f,
            TurnSpeed = 10f,
            ArrivalThreshold = 0.5f,
            UseFlowField = true,
            ViewRadius = 5f,
            SeparationRadius = 1f
        };
    }

    /// <summary>
    /// 障碍物Authoring数据
    /// </summary>
    public struct FlowFieldObstacleAuthoring
    {
        public ObstacleShape Shape;
        public float2 HalfExtents;
        public float Radius;
        public bool IsDynamic;

        public static FlowFieldObstacleAuthoring Default => new FlowFieldObstacleAuthoring
        {
            Shape = ObstacleShape.Circle,
            HalfExtents = new float2(1f, 1f),
            Radius = 1f,
            IsDynamic = false
        };
    }

    /// <summary>
    /// 目标点Authoring数据
    /// </summary>
    public struct FlowFieldGoalAuthoring
    {
        public float2 LocalPosition;
        public float Radius;
        public float InfluenceRadius;
        public int Priority;
        public bool IsActive;

        public static FlowFieldGoalAuthoring Default => new FlowFieldGoalAuthoring
        {
            LocalPosition = float2.zero,
            Radius = 0.5f,
            InfluenceRadius = 5f,
            Priority = 1,
            IsActive = true
        };
    }

    /// <summary>
    /// 流场网格Authoring数据
    /// </summary>
    public struct FlowFieldGridAuthoring
    {
        public float2 WorldBoundsMin;
        public float2 WorldBoundsMax;
        public float CellSize;
        public int UpdateInterval;
        public bool IsStatic;

        public static FlowFieldGridAuthoring Default => new FlowFieldGridAuthoring
        {
            WorldBoundsMin = new float2(-50f, -50f),
            WorldBoundsMax = new float2(50f, 50f),
            CellSize = 1f,
            UpdateInterval = 1,
            IsStatic = false
        };
    }
}