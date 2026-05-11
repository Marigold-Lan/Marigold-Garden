#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

namespace FlowField.Editor
{
    /// <summary>
    /// 流场调试Gizmos绘制器
    /// 在Scene视图中可视化流场网格、方向和状态
    /// </summary>
    [ExecuteInEditMode]
    public class FlowFieldDebugGizmos : MonoBehaviour
    {
        [Header("Display Options")]
        public bool ShowFlowField = true;
        public bool ShowSDFHeatmap = false;
        public bool ShowGridLines = true;
        public bool ShowAgents = true;
        public bool ShowGoals = true;
        public bool ShowObstacles = true;

        [Header("Arrow Settings")]
        public float ArrowScale = 0.5f;
        public float ArrowSpacing = 1.0f;
        public float ArrowHeadSize = 0.15f;

        [Header("Colors")]
        public Color GridColor = new Color(0.3f, 0.3f, 0.3f, 0.5f);
        public Color DirectionColor = new Color(0f, 0.8f, 0f, 0.8f);
        public Color ObstacleColor = new Color(1f, 0f, 0f, 0.5f);
        public Color GoalColor = new Color(0f, 0f, 1f, 0.8f);
        public Color AgentColor = new Color(1f, 1f, 0f, 0.8f);

        [Header("Heatmap Settings")]
        public float HeatmapOpacity = 0.5f;

        private static FlowFieldDebugGizmos instance;

        [MenuItem("FlowField/Debug Gizmos")]
        public static void ToggleDebugGizmos()
        {
            if (instance != null)
            {
                instance.enabled = !instance.enabled;
                Selection.activeObject = instance;
                return;
            }

            GameObject go = new GameObject("FlowField Debug Gizmos");
            instance = go.AddComponent<FlowFieldDebugGizmos>();
            Selection.activeObject = go;
        }

        [DrawGizmo(GizmoType.Selected | GizmoType.Active)]
        private static void DrawGizmo(FlowFieldDebugGizmos gizmos, GizmoType gizmoType)
        {
            if (!Application.isPlaying) return;
            DrawFlowFieldGizmos(gizmos);
        }

        private static void DrawFlowFieldGizmos(FlowFieldDebugGizmos gizmos)
        {
            // 尝试获取流场系统
            var world = UnityEngine.Experimental.Entities.World.DefaultGameObjectInjectionWorld;
            if (world == null) return;

            FlowFieldGrid grid = default;

            try
            {
                var gridSystem = world.GetExistingSystemManaged<FlowField.Systems.FlowFieldGridSystem>();
                if (gridSystem != null)
                {
                    grid = gridSystem.GetGrid();
                }
            }
            catch
            {
                return;
            }

            if (!grid.Distances.IsCreated) return;

            // 绘制网格线
            if (gizmos.ShowGridLines)
            {
                DrawGridLines(grid, gizmos.GridColor);
            }

            // 绘制流场方向箭头
            if (gizmos.ShowFlowField)
            {
                DrawFlowFieldArrows(grid, gizmos.ArrowSpacing, gizmos.ArrowScale, gizmos.ArrowHeadSize, gizmos.DirectionColor);
            }

            // 绘制SDF热力图
            if (gizmos.ShowSDFHeatmap)
            {
                DrawSDFHeatmap(grid, gizmos.HeatmapOpacity);
            }

            // 绘制障碍物
            if (gizmos.ShowObstacles)
            {
                DrawObstacles(gizmos.ObstacleColor);
            }

            // 绘制目标点
            if (gizmos.ShowGoals)
            {
                DrawGoals(gizmos.GoalColor);
            }

            // 绘制代理
            if (gizmos.ShowAgents)
            {
                DrawAgents(gizmos.AgentColor);
            }
        }

        private static void DrawGridLines(FlowFieldGrid grid, Color color)
        {
            Gizmos.color = color;

            float2 origin = grid.Origin;
            float cellSize = grid.CellSize;
            int2 gridSize = grid.GridSize;

            // 垂直线
            for (int x = 0; x <= gridSize.x; x++)
            {
                Vector3 start = new Vector3(origin.x + x * cellSize, 0, origin.y);
                Vector3 end = new Vector3(origin.x + x * cellSize, 0, origin.y + gridSize.y * cellSize);
                Gizmos.DrawLine(start, end);
            }

            // 水平线
            for (int y = 0; y <= gridSize.y; y++)
            {
                Vector3 start = new Vector3(origin.x, 0, origin.y + y * cellSize);
                Vector3 end = new Vector3(origin.x + gridSize.x * cellSize, 0, origin.y + y * cellSize);
                Gizmos.DrawLine(start, end);
            }
        }

        private static void DrawFlowFieldArrows(FlowFieldGrid grid, float spacing, float scale, float headSize, Color color)
        {
            Gizmos.color = color;

            int2 gridSize = grid.GridSize;
            float cellSize = grid.CellSize;
            float2 origin = grid.Origin;

            for (int y = 0; y < gridSize.y; y += (int)math.ceil(spacing))
            {
                for (int x = 0; x < gridSize.x; x += (int)math.ceil(spacing))
                {
                    int idx = y * gridSize.x + x;
                    float2 dir = grid.Directions[idx];

                    float lenSq = math.lengthsq(dir);
                    if (lenSq < 1e-10f) continue;

                    float2 cellCenter = origin + new float2(x + 0.5f, y + 0.5f) * cellSize;
                    Vector3 pos = new Vector3(cellCenter.x, 0, cellCenter.y);
                    Vector3 endPos = pos + new Vector3(dir.x, 0, dir.y) * scale;

                    // 绘制箭头线
                    Gizmos.DrawLine(pos, endPos);

                    // 绘制箭头头部
                    float2 perpDir = new float2(-dir.y, dir.x);
                    float2 head1 = endPos - dir * headSize + perpDir * headSize * 0.5f;
                    float2 head2 = endPos - dir * headSize - perpDir * headSize * 0.5f;
                    Gizmos.DrawLine(endPos, new Vector3(head1.x, 0, head1.y));
                    Gizmos.DrawLine(endPos, new Vector3(head2.x, 0, head2.y));
                }
            }
        }

        private static void DrawSDFHeatmap(FlowFieldGrid grid, float opacity)
        {
            int2 gridSize = grid.GridSize;
            float cellSize = grid.CellSize;
            float2 origin = grid.Origin;

            for (int y = 0; y < gridSize.y; y++)
            {
                for (int x = 0; x < gridSize.x; x++)
                {
                    int idx = y * gridSize.x + x;
                    float distance = grid.Distances[idx];

                    Color color = GetSDFColor(distance, opacity);

                    float2 cellMin = origin + new float2(x, y) * cellSize;
                    Vector3 pos = new Vector3(cellMin.x, -0.01f, cellMin.y);
                    Vector3 size = new Vector3(cellSize, 0.01f, cellSize);

                    Gizmos.color = color;
                    Gizmos.DrawCube(pos + size * 0.5f, size);
                }
            }
        }

        private static Color GetSDFColor(float distance, float opacity)
        {
            if (distance <= 0)
            {
                return new Color(1f, 0f, 0f, opacity);
            }
            else if (distance < 1)
            {
                float t = distance;
                return new Color(1f, t, 0f, opacity);
            }
            else if (distance < 5)
            {
                float t = (distance - 1) / 4f;
                return new Color(1f - t, 1f, 0f, opacity * 0.5f);
            }
            else
            {
                return new Color(0f, 1f, 0f, opacity * 0.3f);
            }
        }

        private static void DrawObstacles(Color color)
        {
            // 需要访问障碍物组件，这里简化为绘制标注
            Gizmos.color = color;
        }

        private static void DrawGoals(Color color)
        {
            Gizmos.color = color;
        }

        private static void DrawAgents(Color color)
        {
            Gizmos.color = color;
        }
    }

    /// <summary>
    /// 流场调试窗口
    /// </summary>
    public class FlowFieldDebugWindow : EditorWindow
    {
        private FlowFieldDebugGizmos debugGizmos;

        [MenuItem("FlowField/Debug Window")]
        public static void ShowWindow()
        {
            GetWindow<FlowFieldDebugWindow>("Flow Field Debug");
        }

        private void OnGUI()
        {
            titleContent = new GUIContent("Flow Field Debug");

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Flow Field Debug Options", EditorStyles.boldLabel);
            EditorGUILayout.Space();

            if (Selection.activeGameObject != null)
            {
                debugGizmos = Selection.activeGameObject.GetComponent<FlowFieldDebugGizmos>();
            }

            if (debugGizmos == null)
            {
                EditorGUILayout.HelpBox("Select a FlowFieldDebugGizmos object to edit options.", MessageType.Info);

                if (GUILayout.Button("Create Debug Gizmos"))
                {
                    FlowFieldDebugGizmos.ToggleDebugGizmos();
                }
                return;
            }

            EditorGUILayout.LabelField("Display", EditorStyles.boldLabel);
            debugGizmos.ShowFlowField = EditorGUILayout.Toggle("Show Flow Field", debugGizmos.ShowFlowField);
            debugGizmos.ShowSDFHeatmap = EditorGUILayout.Toggle("Show SDF Heatmap", debugGizmos.ShowSDFHeatmap);
            debugGizmos.ShowGridLines = EditorGUILayout.Toggle("Show Grid Lines", debugGizmos.ShowGridLines);
            debugGizmos.ShowAgents = EditorGUILayout.Toggle("Show Agents", debugGizmos.ShowAgents);
            debugGizmos.ShowGoals = EditorGUILayout.Toggle("Show Goals", debugGizmos.ShowGoals);
            debugGizmos.ShowObstacles = EditorGUILayout.Toggle("Show Obstacles", debugGizmos.ShowObstacles);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Arrow Settings", EditorStyles.boldLabel);
            debugGizmos.ArrowScale = EditorGUILayout.Slider("Arrow Scale", debugGizmos.ArrowScale, 0.1f, 5f);
            debugGizmos.ArrowSpacing = EditorGUILayout.Slider("Arrow Spacing", debugGizmos.ArrowSpacing, 0.5f, 10f);
            debugGizmos.ArrowHeadSize = EditorGUILayout.Slider("Arrow Head Size", debugGizmos.ArrowHeadSize, 0.05f, 0.5f);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Colors", EditorStyles.boldLabel);
            debugGizmos.GridColor = EditorGUILayout.ColorField("Grid Color", debugGizmos.GridColor);
            debugGizmos.DirectionColor = EditorGUILayout.ColorField("Direction Color", debugGizmos.DirectionColor);
            debugGizmos.ObstacleColor = EditorGUILayout.ColorField("Obstacle Color", debugGizmos.ObstacleColor);
            debugGizmos.GoalColor = EditorGUILayout.ColorField("Goal Color", debugGizmos.GoalColor);
            debugGizmos.AgentColor = EditorGUILayout.ColorField("Agent Color", debugGizmos.AgentColor);

            EditorGUILayout.Space();
            if (GUILayout.Button("Refresh"))
            {
                EditorUtility.SetDirty(debugGizmos);
            }
        }
    }
}
#endif