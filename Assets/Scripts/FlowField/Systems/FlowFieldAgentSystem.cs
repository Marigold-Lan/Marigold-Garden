using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace FlowField.Systems
{
    /// <summary>
    /// 流场代理系统 - 管理代理的寻路和移动
    /// </summary>
    public partial class FlowFieldAgentSystem : SystemBase
    {
        private FlowFieldGridSystem gridSystem;

        protected override void OnCreate()
        {
            gridSystem = World.GetOrCreateSystemManaged<FlowFieldGridSystem>();
        }

        protected override void OnUpdate()
        {
            float deltaTime = SystemAPI.Time.DeltaTime;
            FlowFieldGrid grid = gridSystem.GetGrid();
            int gridVersion = gridSystem.GetVersion();

            // 收集代理数据
            foreach (var agent in SystemAPI.Query<RefRW<FlowFieldAgent>>())
            {
                // 检查是否需要更新寻路
                if (agent.ValueRW.PathVersion < gridVersion)
                {
                    agent.ValueRW.State = AgentState.Seeking;
                    agent.ValueRW.PathVersion = gridVersion;
                }

                // 如果不在寻路状态，跳过
                if (agent.ValueRO.State != AgentState.Seeking)
                    continue;

                // 获取流场方向
                int2 cell = grid.WorldToCellSafe(agent.ValueRO.Position);
                if (!grid.IsValidCell(cell))
                {
                    agent.ValueRW.State = AgentState.NoPath;
                    continue;
                }

                int cellIdx = grid.CellToIndex(cell);
                float2 direction = grid.Directions[cellIdx];

                if (math.lengthsq(direction) < 1e-10f)
                {
                    agent.ValueRW.State = AgentState.NoPath;
                    continue;
                }

                // 归一化方向
                direction = math.normalize(direction);

                // 计算速度
                float2 newVelocity = direction * agent.ValueRO.MoveSpeed;

                // 应用转向
                float currentAngle = math.atan2(agent.ValueRO.Velocity.y, agent.ValueRO.Velocity.x);
                float desiredAngle = math.atan2(direction.y, direction.x);

                float angleDiff = NormalizeAngle(desiredAngle - currentAngle);
                float maxTurn = agent.ValueRO.TurnSpeed * deltaTime;
                float newAngle = currentAngle + math.clamp(angleDiff, -maxTurn, maxTurn);

                float2 smoothDir = new float2(math.cos(newAngle), math.sin(newAngle));
                newVelocity = smoothDir * agent.ValueRO.MoveSpeed;

                // 更新位置
                float2 newPosition = agent.ValueRO.Position + newVelocity * deltaTime;

                // 检查是否到达目标
                if (agent.ValueRO.CurrentGoal != Entity.Null)
                {
                    if (SystemAPI.HasComponent<FlowFieldGoal>(agent.ValueRO.CurrentGoal))
                    {
                        FlowFieldGoal goal = SystemAPI.GetComponent<FlowFieldGoal>(agent.ValueRO.CurrentGoal);
                        float dist = math.length(newPosition - goal.Position);
                        if (dist <= agent.ValueRO.ArrivalThreshold)
                        {
                            agent.ValueRW.State = AgentState.Arrived;
                            newPosition = goal.Position;
                        }
                    }
                }

                // 更新组件
                agent.ValueRW.Position = newPosition;
                agent.ValueRW.Velocity = newVelocity;
                agent.ValueRW.RemainingPathDistance = grid.GoalDistances[cellIdx];
            }
        }

        [GenerateTestsForBurstCompatibility]
        private static float NormalizeAngle(float angle)
        {
            while (angle > math.PI) angle -= 2 * math.PI;
            while (angle < -math.PI) angle += 2 * math.PI;
            return angle;
        }
    }

    /// <summary>
    /// 批量代理移动系统 - 使用Jobs实现大规模代理并行移动
    /// </summary>
    public partial class BatchAgentMoveSystem : SystemBase
    {
        private FlowFieldGridSystem gridSystem;

        protected override void OnCreate()
        {
            gridSystem = World.GetOrCreateSystemManaged<FlowFieldGridSystem>();
        }

        protected override void OnUpdate()
        {
            float deltaTime = SystemAPI.Time.DeltaTime;

            // 收集所有代理数据
            NativeArray<float2> positions = new NativeArray<float2>(4096, Allocator.TempJob);
            NativeArray<float> moveSpeeds = new NativeArray<float>(4096, Allocator.TempJob);
            NativeArray<float> turnSpeeds = new NativeArray<float>(4096, Allocator.TempJob);
            NativeArray<float2> velocities = new NativeArray<float2>(4096, Allocator.TempJob);
            NativeArray<int> pathVersions = new NativeArray<int>(4096, Allocator.TempJob);
            NativeArray<int> states = new NativeArray<int>(4096, Allocator.TempJob);
            NativeArray<Entity> goals = new NativeArray<Entity>(4096, Allocator.TempJob);
            NativeArray<float> arrivalThresholds = new NativeArray<float>(4096, Allocator.TempJob);
            NativeArray<float2> goalPositions = new NativeArray<float2>(4096, Allocator.TempJob);

            int agentCount = 0;

            int queryIndex = 0;
            foreach (var agent in SystemAPI.Query<RefRO<FlowFieldAgent>>())
            {
                if (queryIndex >= 4096) break;

                positions[agentCount] = agent.ValueRO.Position;
                moveSpeeds[agentCount] = agent.ValueRO.MoveSpeed;
                turnSpeeds[agentCount] = agent.ValueRO.TurnSpeed;
                velocities[agentCount] = agent.ValueRO.Velocity;
                pathVersions[agentCount] = agent.ValueRO.PathVersion;
                states[agentCount] = (int)agent.ValueRO.State;
                arrivalThresholds[agentCount] = agent.ValueRO.ArrivalThreshold;

                if (agent.ValueRO.CurrentGoal != Entity.Null &&
                    SystemAPI.HasComponent<FlowFieldGoal>(agent.ValueRO.CurrentGoal))
                {
                    goalPositions[agentCount] = SystemAPI.GetComponent<FlowFieldGoal>(agent.ValueRO.CurrentGoal).Position;
                }
                else
                {
                    goalPositions[agentCount] = float2.zero;
                }

                agentCount++;
                queryIndex++;
            }

            if (agentCount == 0)
            {
                positions.Dispose();
                moveSpeeds.Dispose();
                turnSpeeds.Dispose();
                velocities.Dispose();
                pathVersions.Dispose();
                states.Dispose();
                goals.Dispose();
                arrivalThresholds.Dispose();
                goalPositions.Dispose();
                return;
            }

            FlowFieldGrid grid = gridSystem.GetGrid();
            int gridVersion = gridSystem.GetVersion();

            // 创建移动Job
            var moveJob = new BatchAgentMoveJob
            {
                Positions = positions,
                MoveSpeeds = moveSpeeds,
                TurnSpeeds = turnSpeeds,
                Velocities = velocities,
                PathVersions = pathVersions,
                OutPositions = positions,
                OutVelocities = velocities,
                OutStates = states,
                GoalPositions = goalPositions,
                ArrivalThresholds = arrivalThresholds,
                CellTypes = grid.CellTypes,
                Directions = grid.Directions,
                GridSize = grid.GridSize,
                CellSize = grid.CellSize,
                Origin = grid.Origin,
                DeltaTime = deltaTime,
                GridVersion = gridVersion,
                AgentCount = agentCount
            };

            moveJob.Run();

            // 写回结果
            int idx = 0;
            foreach (var agent in SystemAPI.Query<RefRW<FlowFieldAgent>>())
            {
                if (idx >= agentCount) break;

                agent.ValueRW.Position = positions[idx];
                agent.ValueRW.Velocity = velocities[idx];
                agent.ValueRW.State = (AgentState)states[idx];
                agent.ValueRW.PathVersion = gridVersion;

                // 检查到达
                if (agent.ValueRO.State == AgentState.Seeking &&
                    agent.ValueRO.CurrentGoal != Entity.Null)
                {
                    float dist = math.length(agent.ValueRO.Position - goalPositions[idx]);
                    if (dist <= agent.ValueRO.ArrivalThreshold)
                    {
                        agent.ValueRW.State = AgentState.Arrived;
                    }
                }

                idx++;
            }

            // 清理
            positions.Dispose();
            moveSpeeds.Dispose();
            turnSpeeds.Dispose();
            velocities.Dispose();
            pathVersions.Dispose();
            states.Dispose();
            goals.Dispose();
            arrivalThresholds.Dispose();
            goalPositions.Dispose();
        }

        [BurstCompile]
        struct BatchAgentMoveJob : IJob
        {
            [ReadOnly] public NativeArray<float2> Positions;
            [ReadOnly] public NativeArray<float> MoveSpeeds;
            [ReadOnly] public NativeArray<float> TurnSpeeds;
            [ReadOnly] public NativeArray<float2> Velocities;
            [ReadOnly] public NativeArray<int> PathVersions;
            [ReadOnly] public NativeArray<float2> GoalPositions;
            [ReadOnly] public NativeArray<float> ArrivalThresholds;

            [ReadOnly] public NativeArray<byte> CellTypes;
            [ReadOnly] public NativeArray<float2> Directions;

            public NativeArray<float2> OutPositions;
            public NativeArray<float2> OutVelocities;
            public NativeArray<int> OutStates;

            public int2 GridSize;
            public float CellSize;
            public float2 Origin;
            public float DeltaTime;
            public int GridVersion;
            public int AgentCount;

            public void Execute()
            {
                for (int i = 0; i < AgentCount; i++)
                {
                    int state = PathVersions[i] < GridVersion ? 1 : PathVersions[i] == GridVersion ? (int)AgentState.Seeking : (int)AgentState.NoPath;

                    if (state != (int)AgentState.Seeking)
                    {
                        OutPositions[i] = Positions[i];
                        OutVelocities[i] = Velocities[i];
                        OutStates[i] = state;
                        continue;
                    }

                    float2 position = Positions[i];
                    float speed = MoveSpeeds[i];
                    float turnSpeed = TurnSpeeds[i];
                    float2 velocity = Velocities[i];

                    // 获取方向
                    int2 cell = GetCell(position);
                    if (!IsValidCell(cell))
                    {
                        OutPositions[i] = position;
                        OutVelocities[i] = float2.zero;
                        OutStates[i] = (int)AgentState.NoPath;
                        continue;
                    }

                    int cellIdx = cell.y * GridSize.x + cell.x;
                    float2 direction = Directions[cellIdx];

                    float lenSq = math.lengthsq(direction);
                    if (lenSq < 1e-10f)
                    {
                        OutPositions[i] = position;
                        OutVelocities[i] = float2.zero;
                        OutStates[i] = (int)AgentState.NoPath;
                        continue;
                    }

                    direction = direction / math.sqrt(lenSq);

                    // 转向
                    float currentAngle = math.atan2(velocity.y, velocity.x);
                    float desiredAngle = math.atan2(direction.y, direction.x);

                    float angleDiff = NormalizeAngle(desiredAngle - currentAngle);
                    float maxTurn = turnSpeed * DeltaTime;
                    float newAngle = currentAngle + math.clamp(angleDiff, -maxTurn, maxTurn);

                    float2 newDir = new float2(math.cos(newAngle), math.sin(newAngle));
                    float2 newVelocity = newDir * speed;
                    float2 newPosition = position + newVelocity * DeltaTime;

                    // 边界检查
                    float2 boundsMin = Origin;
                    float2 boundsMax = Origin + new float2(GridSize.x * CellSize, GridSize.y * CellSize);
                    newPosition.x = math.clamp(newPosition.x, boundsMin.x, boundsMax.x);
                    newPosition.y = math.clamp(newPosition.y, boundsMin.y, boundsMax.y);

                    OutPositions[i] = newPosition;
                    OutVelocities[i] = newVelocity;
                    OutStates[i] = (int)AgentState.Seeking;
                }
            }

            [GenerateTestsForBurstCompatibility]
            private int2 GetCell(float2 worldPos)
            {
                int2 cell = new int2(
                    (int)math.floor((worldPos.x - Origin.x) / CellSize),
                    (int)math.floor((worldPos.y - Origin.y) / CellSize)
                );
                cell.x = math.clamp(cell.x, 0, GridSize.x - 1);
                cell.y = math.clamp(cell.y, 0, GridSize.y - 1);
                return cell;
            }

            [GenerateTestsForBurstCompatibility]
            private bool IsValidCell(int2 cell)
            {
                return (uint)cell.x < (uint)GridSize.x && (uint)cell.y < (uint)GridSize.y;
            }

            [GenerateTestsForBurstCompatibility]
            private float NormalizeAngle(float angle)
            {
                while (angle > math.PI) angle -= 2 * math.PI;
                while (angle < -math.PI) angle += 2 * math.PI;
                return angle;
            }
        }
    }
}