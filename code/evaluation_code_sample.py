# Example of Fitt's law statistics added to evaluation module

def fitts_id(D, R):
    """Shannon ID = log2(D/(2R) + 1). R is target radius; 2R is target width."""
    return math.log2(D / (2 * R) + 1) if R > 0 else 0.0

# In main function, when generating aggregate outputs: 
for base_pid in sorted(participants.keys()):
    pid      = f"{base_pid}_{args.model_type}"
    p_folder = RESULTS_DIR / f"participant_{pid}"
    for (tid, radius), cond in sorted(fitts_conditions.items(),
                                        key=lambda kv: (kv[0][0], kv[0][1])):
        W  = cond["width"]
        R  = cond["target_radius"]
        D  = centerline_lengths.get(tid, 0.0)
        ID = fitts_id(D, R)

        summary_path = p_folder / condition_folder_name(tid, radius) / "results_summary.json"
        if not summary_path.exists():
            continue

        with open(summary_path) as f:
            sm = json.load(f)