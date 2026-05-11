using Unity.Entities;
using Unity.Mathematics;

namespace FlowField
{
    /// <summary>
    /// 障碍物组件 - 标记为障碍物的实体
    /// 支持圆形和矩形两种形状
    /// </summary>
    public struct FlowFieldObstacle : IComponentData
    {
        // ==================== 形状参数 ====================
        public ObstacleShape Shape;
        public float Radius;           // 圆形半径或等效半径
        public float2 HalfExtents;      // 矩形半尺寸

        // ==================== 位置 ====================
        public float2 Position;

        // ==================== 状态 ====================
        public bool IsDynamic;
        public int GridVersion;
        public bool IsDirty;

        /// <summary>
        /// 创建圆形障碍物
        /// </summary>
        public static FlowFieldObstacle CreateCircle(float2 position, float radius, bool isDynamic = false)
        {
            return new FlowFieldObstacle
            {
                Shape = ObstacleShape.Circle,
                Radius = radius,
                HalfExtents = float2.zero,
                Position = position,
                IsDynamic = isDynamic,
                GridVersion = -1,
                IsDirty = true
            };
        }

        /// <summary>
        /// 创建矩形障碍物
        /// </summary>
        public static FlowFieldObstacle CreateBox(float2 position, float2 halfExtents, bool isDynamic = false)
        {
            float maxDim = math.max(halfExtents.x, halfExtents.y);
            return new FlowFieldObstacle
            {
                Shape = ObstacleShape.Box,
                Radius = math.length(halfExtents),
                HalfExtents = halfExtents,
                Position = position,
                IsDynamic = isDynamic,
                GridVersion = -1,
                IsDirty = true
            };
        }

        /// <summary>
        /// 计算障碍物的SDF值
        /// </summary>
        public float CalculateSDF(float2 point)
        {
            switch (Shape)
            {
                case ObstacleShape.Circle:
                    return FlowFieldMath.SDFCircle(point, Position, Radius);
                case ObstacleShape.Box:
                    return FlowFieldMath.SDFBox(point, Position, HalfExtents);
                default:
                    return float.MaxValue;
            }
        }

        /// <summary>
        /// 计算障碍物的SDF梯度
        /// </summary>
        public float2 CalculateSDFGradient(float2 point)
        {
            switch (Shape)
            {
                case ObstacleShape.Circle:
                    return FlowFieldMath.SDFCircleGradient(point, Position);
                case ObstacleShape.Box:
                    return FlowFieldMath.SDFBoxGradient(point, Position, HalfExtents);
                default:
                    return float2.zero;
            }
        }

        /// <summary>
        /// 检查点是否在障碍物内部
        /// </summary>
        public bool ContainsPoint(float2 point)
        {
            return CalculateSDF(point) < 0f;
        }

        /// <summary>
        /// 获取障碍物的边界
        /// </summary>
        public float4 GetBounds()
        {
            switch (Shape)
            {
                case ObstacleShape.Circle:
                    return new float4(
                        Position.x - Radius, Position.y - Radius,
                        Position.x + Radius, Position.y + Radius
                    );
                case ObstacleShape.Box:
                    return new float4(
                        Position.x - HalfExtents.x, Position.y - HalfExtents.y,
                        Position.x + HalfExtents.x, Position.y + HalfExtents.y
                    );
                default:
                    return float4.zero;
            }
        }
    }

    /// <summary>
    /// 静态障碍物标记 - 用于标识不需要每帧更新的障碍物
    /// </summary>
    public struct FlowFieldStaticObstacle : IComponentData
    {
        public float2 MinBound;
        public float2 MaxBound;
    }
}