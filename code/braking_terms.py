# 1: Potential-well formulation -- penalty becomes larger near the target, but fades with distance (continuous)
# This was the braking term used for the final version of the simulation; least failures and most like human behavior near button

w_precision = weights.get('goal_precision', 0.0)
target_radius = weights.get('target_radius', 0.01)
if w_precision > 0.0:
    # both nc0 and nc1 are signal-dependent noise coefficients
    nc0 = weights.get('nc0', 0.2)
    nc1 = weights.get('nc1', 0.02)
    # predicted velocity-scatter variance at each node is (nc * v)^2
    scatter_var = (nc0**2 + nc1**2) * (vx**2 + vy**2)
    # physical (x, y) of the path endpoint, no arc-length gating needed
    p_end = ref_path(ref_path.total_length)
    x_target, y_target = float(p_end[0]), float(p_end[1])
    # squared Euclidean distance from every predicted position to the target
    dist_sq = (px - x_target)**2 + (py - y_target)**2
    # potential-well
    r2 = target_radius**2
    per_node_penalty = scatter_var / (dist_sq + r2)
    goal_cost = w_precision * np.sum(per_node_penalty)
else:
    goal_cost = 0.0




# Previous terms I tested: 

# 2: Strict speed-accuracy tradeoff
# Failures: Many trajectories stopped before the target button because the 1 / R^2 scaling was too high
w_precision = weights.get('goal_precision', 0.0)
target_radius = weights.get('target_radius', 0.01)
if w_precision > 0.0:
    s_end = ref_path.total_length
    #both nc0 and nc1 are signal-dependent
    nc0 = weights.get('nc0', 0.2)
    nc1 = weights.get('nc1', 0.02)

    scatter_var = (nc0**2 + nc1**2) * (vx**2 + vy**2)
    at_goal = s_traj >= s_end
    scatter_at_goal = np.sum(np.where(at_goal, scatter_var, 0.0))
    goal_cost = w_precision * scatter_at_goal / (target_radius**2)
else:
    goal_cost = 0.0


# 3: Huber Bound: modification of strict speed-accuracy tradeoff
# Failures: Timed-out simulations for cursors that stopped just outside the button
w_precision = weights.get('goal_precision', 0.0)
target_radius = weights.get('target_radius', 0.01)
if w_precision > 0.0:
    s_end = ref_path.total_length
    nc0 = weights.get('nc0', 0.2)
    nc1 = weights.get('nc1', 0.02)

    # normalized scatter-to-radius ratio per horizon node
    #adding 1e-8 to prevent undefined gradient term if v == 0
    scatter_std = np.sqrt(
        (nc0**2 + nc1**2) * (vx**2 + vy**2) + 1e-8 
    )
    z = scatter_std / target_radius 

    # Piecewise function: quadratic when z <= 1, linear when z > 1.
    huber_z = np.where(z <= 1.0, z**2, 2.0 * z - 1.0)

    at_goal = s_traj >= s_end
    goal_cost = w_precision * np.sum(np.where(at_goal, huber_z, 0.0))
else:
    goal_cost = 0.0


# 4: Exponential Saturation
# Failures: Shooting past the target, and/or rebounding back at full speed
w_precision = weights.get('goal_precision', 0.0)
target_radius = weights.get('target_radius', 0.01)
if w_precision > 0.0:
    s_end = ref_path.total_length
    nc0 = weights.get('nc0', 0.2)
    nc1 = weights.get('nc1', 0.02)
    # scatter_var[k] = (nc0^2 + nc1^2) * ||v_k||^2
    scatter_var = (nc0**2 + nc1**2) * (vx**2 + vy**2)
    at_goal = s_traj >= s_end

    # per-node penalty in [0, 1): 1 - exp(-scatter_var / R^2)
    r2 = target_radius ** 2
    per_node_penalty = 1.0 - np.exp(-scatter_var / r2)

    # sum only over nodes that have reached the goal arc-length
    goal_cost = w_precision * np.sum(np.where(at_goal, per_node_penalty, 0.0))
else:
    goal_cost = 0.0