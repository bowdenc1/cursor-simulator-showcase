# Generalized function for wide-to-narrow and narrow-to-wide tunnels (tunnels without one static width)

def convert_constraints_to_corridor_bounds(
    constraints: Optional[ConstraintConfig],
    reference_path,
    default_margin: float = 0.0,
    screen_width: Optional[float] = None,
    screen_height: Optional[float] = None
) -> Optional[Tuple[Callable, Callable]]:
    if constraints is None:
        return None

    margin = getattr(constraints, 'default_margin', default_margin)

    # Define sampling grid up front — needed for per-point width interpolation
    num_samples = max(50, int(reference_path.total_length / 0.01))
    s_samples = np.linspace(0, reference_path.total_length, num_samples)

    half_widths: List[float] = []
    for region in constraints.regions:
        geom = region.geometry
        if isinstance(geom, PathConstraint) and geom.width is not None:
            region_margin = getattr(region, 'margin', None)
            if region_margin is None:
                region_margin = margin
            w = geom.width
            if isinstance(w, list):
                w_arr = np.array(w, dtype=float)
                fracs = np.linspace(0, 1, len(w_arr))
                s_fracs = s_samples / reference_path.total_length
                w_interp = np.interp(s_fracs, fracs, w_arr)
                hw = np.maximum(0.0, w_interp / 2.0 - region_margin)
                half_widths.append(hw)
            else:
                half_widths.append(max(0.0, float(w) / 2.0 - region_margin))
    #etc. 