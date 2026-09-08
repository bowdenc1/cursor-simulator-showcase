#New code that replaced hard stop mechanism
# Termination: DWELL_S of consecutive samples inside the target
# (stands in for the human click latency; the pointing data show
# ~0.3 s median from final target entry to click).
dwell_required = int(round(self.dwell_s / self.interval))
dwell_steps = 0
for step in range(max_steps):
    dist_to_target = np.linalg.norm(cursor_pos - final_target)
    if dist_to_target < target_radius:
        dwell_steps += 1
        if dwell_steps >= dwell_required:
            break
    else:
        dwell_steps = 0

    tunnel_path = waypoints_norm
    model_input = SteeringModelInput(
        state_cog=(
            float(cursor_pos[0]),
            float(cursor_pos[1]),
            float(cursor_vel[0]),
            float(cursor_vel[1])
        ),
        bump=BumpParams(
            pred_horizon=self.pred_horizon,
            Tp=self.tp,
            nc=self.nc
        ),
        #etc. 
    )