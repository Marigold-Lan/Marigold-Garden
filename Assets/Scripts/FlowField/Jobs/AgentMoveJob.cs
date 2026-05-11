using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace FlowField.Jobs
{
    /// <summary>
    /// 代理移动Job - 基于流场方向移动代理
    /// 包含避障、速度平滑等行为
    /// </summary>
    [BurstCompile(OptimizeFor = OptimizeFor.Throughput)]
    public struct AgentMoveJob : IJobParallelFor
    {
        // ==================== 代理数据 ====================
        [ReadOnly] public NativeArray<float2> Positions;
        [ReadOnly] public NativeArray<float> MoveSpeeds;
        [ReadOnly] public NativeArray<float> TurnSpeeds;
        [ReadOnly] public NativeArray<float2> Velocities;

        [WriteOnly] public NativeArray<float2> OutPositions;
        [WriteOnly] public NativeArray<float2> OutVelocities;

        // ==================== 流场数据 ====================
        [ReadOnly] public NativeArray<float2> FlowDirections;
        [ReadOnly] public NativeArray<byte> CellTypes;

        // ==================== 空间哈希（邻居查询）====================
        [ReadOnly] public NativeArray<int> NeighborEntities;
        [ReadOnly] public NativeArray<int> NeighborOffsets;
        [ReadOnly] public NativeArray<float2> NeighborPositions;

        // ==================== 配置 ====================
        public int2 GridSize;
        public float CellSize;
        public float2 Origin;
        public float SeparationWeight;
        public float NeighborRepulsionRadius;
        public float MaxSpeed;
        public float DeltaTime;

        public void Execute(int index)
        {
            float2 position = Positions[index];
            float speed = MoveSpeeds[index];
            float turnSpeed = TurnSpeeds[index];
            float2 velocity = Velocities[index];

            // ==================== 1. 从流场获取目标方向 ====================
            float2 targetDirection = GetFlowDirection(position);

            // ==================== 2. 邻居排斥力 ====================
            float2 separationForce = float2.zero;
            int neighborStart = NeighborOffsets[index];
            int neighborEnd = (index + 1 < NeighborOffsets.Length)
                ? NeighborOffsets[index + 1]
                : NeighborEntities.Length;

            int neighborCount = 0;
            for (int i = neighborStart; i < neighborEnd && i < NeighborEntities.Length; i++)
            {
                int neighborId = NeighborEntities[i];
                if (neighborId == index) continue; // 排除自己

                // 假设邻居位置按某种方式存储或可查询
                float2 neighborPos = NeighborPositions[i];

                float2 offset = position - neighborPos;
                float distSq = math.lengthsq(offset);

                if (distSq > 0f && distSq < NeighborRepulsionRadius * NeighborRepulsionRadius)
                {
                    float dist = math.sqrt(distSq);
                    float2 away = offset / dist;
                    float force = 1f - (dist / NeighborRepulsionRadius);
                    separationForce += away * force;
                    neighborCount++;
                }
            }

            // ==================== 3. 计算最终方向 ====================
            float2 desiredDirection = targetDirection + separationForce * SeparationWeight;
            float lenSq = math.lengthsq(desiredDirection);

            if (lenSq > 1e-10f)
            {
                desiredDirection = desiredDirection / math.sqrt(lenSq);
            }
            else
            {
                desiredDirection = targetDirection;
            }

            // ==================== 4. 平滑转向 ====================
            float currentAngle = math.atan2(velocity.y, velocity.x);
            float desiredAngle = math.atan2(desiredDirection.y, desiredDirection.x);

            float angleDiff = NormalizeAngleDifference(desiredAngle - currentAngle);
            float maxTurn = turnSpeed * DeltaTime;
            float newAngle = currentAngle + math.clamp(angleDiff, -maxTurn, maxTurn);

            float2 newDirection = new float2(math.cos(newAngle), math.sin(newAngle));

            // ==================== 5. 更新速度 ====================
            float2 newVelocity = newDirection * speed;
            newVelocity = math.clamp(newVelocity, new float2(-MaxSpeed), new float2(MaxSpeed));

            // ==================== 6. 更新位置 ====================
            float2 newPosition = position + newVelocity * DeltaTime;

            // 边界检查
            float2 boundsMin = Origin;
            float2 boundsMax = Origin + new float2(GridSize.x * CellSize, GridSize.y * CellSize);
            newPosition.x = math.clamp(newPosition.x, boundsMin.x, boundsMax.x);
            newPosition.y = math.clamp(newPosition.y, boundsMin.y, boundsMax.y);

            OutPositions[index] = newPosition;
            OutVelocities[index] = newVelocity;
        }

        [GenerateTestsForBurstCompatibility]
        private float2 GetFlowDirection(float2 worldPos)
        {
            int2 cell = WorldToCell(worldPos);
            if (!IsValidCell(cell)) return float2.zero;

            int idx = cell.y * GridSize.x + cell.x;
            return FlowDirections[idx];
        }

        [GenerateTestsForBurstCompatibility]
        private int2 WorldToCell(float2 worldPos)
        {
            int2 cell = new int2(
                (int)math.floor((worldPos.x - Origin.x) / CellSize),
                (int)math.floor((worldPos.y - Origin.y) / CellSize)
            );
            return cell;
        }

        [GenerateTestsForBurstCompatibility]
        private bool IsValidCell(int2 cell)
        {
            return (uint)cell.x < (uint)GridSize.x && (uint)cell.y < (uint)GridSize.y;
        }

        [GenerateTestsForBurstCompatibility]
        private float NormalizeAngleDifference(float diff)
        {
            while (diff > math.PI) diff -= 2 * math.PI;
            while (diff < -math.PI) diff += 2 * math.PI;
            return diff;
        }
    }

    /// <summary>
    /// 简化代理移动Job - 用于测试或性能敏感场景
    /// 只使用流场方向，不包含避障
    /// </summary>
    [BurstCompile(OptimizeFor = OptimizeFor.Throughput)]
    public struct SimpleAgentMoveJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float2> Positions;
        [ReadOnly] public NativeArray<float> MoveSpeeds;

        [WriteOnly] public NativeArray<float2> OutPositions;

        [ReadOnly] public NativeArray<float2> FlowDirections;
        [ReadOnly] public NativeArray<byte> CellTypes;

        public int2 GridSize;
        public float CellSize;
        public float2 Origin;
        public float DeltaTime;

        public void Execute(int index)
        {
            float2 position = Positions[index];
            float speed = MoveSpeeds[index];

            // 获取流场方向
            float2 direction = GetFlowDirection(position);

            // 移动
            float2 newPosition = position + direction * speed * DeltaTime;

            // 边界检查
            float2 boundsMin = Origin;
            float2 boundsMax = Origin + new float2(GridSize.x * CellSize, GridSize.y * CellSize);
            newPosition.x = math.clamp(newPosition.x, boundsMin.x, boundsMax.x);
            newPosition.y = math.clamp(newPosition.y, boundsMin.y, boundsMax.y);

            OutPositions[index] = newPosition;
        }

        [GenerateTestsForBurstCompatibility]
        private float2 GetFlowDirection(float2 worldPos)
        {
            int2 cell = new int2(
                (int)math.floor((worldPos.x - Origin.x) / CellSize),
                (int)math.floor((worldPos.y - Origin.y) / CellSize)
            );

            if ((uint)cell.x >= (uint)GridSize.x || (uint)cell.y >= (uint)GridSize.y)
                return float2.zero;

            int idx = cell.y * GridSize.x + cell.x;
            return FlowDirections[idx];
        }
    }

    /// <summary>
    /// 到达检测Job - 检测代理是否到达目标
    /// </summary>
    [BurstCompile(OptimizeFor = OptimizeFor.Throughput)]
    public struct ArrivalCheckJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float2> Positions;
        [ReadOnly] public NativeArray<float2> GoalPositions;
        [ReadOnly] public NativeArray<float> ArrivalThresholds;

        [WriteOnly] public NativeArray<byte> ArrivalResults; // 0=未到达, 1=到达

        public void Execute(int index)
        {
            float dist = math.length(Positions[index] - GoalPositions[index]);
            ArrivalResults[index] = (dist <= ArrivalThresholds[index]) ? (byte)1 : (byte)0;
        }
    }

    /// <summary>
    /// 目标选择Job - 代理选择最近的目标
    /// </summary>
    [BurstCompile(OptimizeFor = OptimizeFor.Throughput)]
    public struct GoalSelectionJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float2> AgentPositions;
        [ReadOnly] public NativeArray<int> AgentCurrentGoals;

        [ReadOnly] public NativeArray<float2> GoalPositions;
        [ReadOnly] public NativeArray<int> GoalPriorities;
        [ReadOnly] public NativeArray<bool> GoalActive;

        [WriteOnly] public NativeArray<int> SelectedGoals;

        public int GoalCount;

        public void Execute(int index)
        {
            float2 agentPos = AgentPositions[index];

            int bestGoal = -1;
            float bestScore = float.MaxValue;

            for (int i = 0; i < GoalCount; i++)
            {
                if (!GoalActive[i]) continue;

                float dist = math.length(agentPos - GoalPositions[i]);
                float score = dist / math.max(1, GoalPriorities[i]);

                if (score < bestScore)
                {
                    bestScore = score;
                    bestGoal = i;
                }
            }

            SelectedGoals[index] = bestGoal;
        }
    }
}