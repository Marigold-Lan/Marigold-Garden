using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace FlowField.Systems
{
    /// <summary>
    /// 流场调试可视化系统
    /// 在Editor模式下显示流场方向和SDF热力图
    /// </summary>
    public partial class FlowFieldDebugSystem : SystemBase
    {
#if UNITY_EDITOR
        public bool ShowFlowField = true;
        public bool ShowSDFHeatmap = false;
        public bool ShowGrid = true;
        public bool ShowAgents = true;
        public bool ShowGoals = true;
        public bool ShowObstacles = true;
        public float ArrowScale = 1f;
        public float ArrowSpacing = 1f;

        private FlowFieldGridSystem gridSystem;

        protected override void OnCreate()
        {
            gridSystem = World.GetOrCreateSystemManaged<FlowFieldGridSystem>();
        }

        protected override void OnUpdate()
        {
            // 调试绘制在OnGUI或Gizmos中完成
        }

        /// <summary>
        /// 获取调试信息字符串
        /// </summary>
        public string GetDebugInfo()
        {
            FlowFieldGrid grid = gridSystem.GetGrid();
            int agentCount = 0;
            int obstacleCount = 0;
            int goalCount = 0;

            foreach (var _ in SystemAPI.Query<RefRO<FlowFieldAgent>>()) { agentCount++; }

            foreach (var _ in SystemAPI.Query<RefRO<FlowFieldObstacle>>()) { obstacleCount++; }

            foreach (var _ in SystemAPI.Query<RefRO<FlowFieldGoal>>()) { goalCount++; }

            return $"Flow Field Debug Info:\n" +
                   $"Grid Size: {grid.GridSize.x}x{grid.GridSize.y}\n" +
                   $"Cell Size: {grid.CellSize}\n" +
                   $"Version: {grid.Version}\n" +
                   $"Agents: {agentCount}\n" +
                   $"Obstacles: {obstacleCount}\n" +
                   $"Goals: {goalCount}";
        }

#endif
    }

    /// <summary>
    /// 调试绘制辅助类
    /// </summary>
    public static class FlowFieldDebug
    {
#if UNITY_EDITOR
        /// <summary>
        /// 获取流场网格用于编辑器绘制
        /// </summary>
        public static FlowFieldGrid GetGrid(World world)
        {
            var gridSystem = world.GetOrCreateSystemManaged<FlowFieldGridSystem>();
            return gridSystem.GetGrid();
        }

        /// <summary>
        /// 计算SDF热力图颜色
        /// </summary>
        public static UnityEngine.Color SDFToColor(float distance)
        {
            if (distance <= 0)
            {
                // 障碍物 - 红色
                return new UnityEngine.Color(1f, 0f, 0f, 0.8f);
            }
            else if (distance < 1)
            {
                // 接近障碍物 - 黄色到橙色
                float t = distance;
                return new UnityEngine.Color(1f, t, 0f, 0.6f);
            }
            else if (distance < 5)
            {
                // 安全区域 - 绿色渐变
                float t = (distance - 1) / 4f;
                return new UnityEngine.Color(1f - t, 1f, 0f, 0.4f);
            }
            else
            {
                // 远距离 - 蓝色
                float t = math.saturate((distance - 5) / 10f);
                return new UnityEngine.Color(0f, 1f - t, t, 0.3f);
            }
        }

        /// <summary>
        /// 方向转颜色（用于显示流场方向）
        /// </summary>
        public static UnityEngine.Color DirectionToColor(float2 direction)
        {
            // 使用色相环表示方向
            float angle = math.atan2(direction.y, direction.x);
            float hue = (angle + math.PI) / (2 * math.PI);

            float r = math.saturate(math.abs(hue * 6 - 3) - 1);
            float g = math.saturate(2 - math.abs(hue * 6 - 2));
            float b = math.saturate(2 - math.abs(hue * 6 - 4));

            return new UnityEngine.Color(r, g, b, 0.8f);
        }

        /// <summary>
        /// 方向转灰度颜色
        /// </summary>
        public static UnityEngine.Color DirectionToGrayscale(float2 direction)
        {
            float len = math.length(direction);
            if (len < 1e-10f)
                return new UnityEngine.Color(0.5f, 0.5f, 0.5f, 0.3f);

            float angle = math.atan2(direction.y, direction.x);
            float normalized = (angle + math.PI) / (2 * math.PI);

            return new UnityEngine.Color(normalized, normalized, normalized, 0.6f);
        }
#endif
    }

    /// <summary>
    /// 调试选项配置
    /// </summary>
    public struct FlowFieldDebugOptions
    {
        public bool ShowFlowField;
        public bool ShowSDFHeatmap;
        public bool ShowGrid;
        public bool ShowAgents;
        public bool ShowGoals;
        public bool ShowObstacles;
        public bool ShowCells;
        public float ArrowScale;
        public float ArrowSpacing;
        public float HeatmapOpacity;

        public static FlowFieldDebugOptions Default => new FlowFieldDebugOptions
        {
            ShowFlowField = true,
            ShowSDFHeatmap = false,
            ShowGrid = true,
            ShowAgents = true,
            ShowGoals = true,
            ShowObstacles = true,
            ShowCells = false,
            ArrowScale = 1f,
            ArrowSpacing = 1f,
            HeatmapOpacity = 0.5f
        };
    }
}