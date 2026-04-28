# One-command reproduction of the paper's headline result.
#
#   make test       Run all pytest tests (~7 s).
#   make ablation   Run 40-trial acoustic on/off grid -> CSVs.
#   make figure     Render the trajectory comparison PNG.
#   make stats      Run paired Wilcoxon on the saved CSV.
#   make headline   Do all of the above end to end.
#   make paper-pdf  Compile the LaTeX (requires pdflatex).
#   make clean      Remove generated artifacts.

PY        := python
RUNS_DIR  := experiments/runs
FIG_DIR   := paper/figures
SEEDS     := 0 1 2 3 4 5 6 7 8 9
SCENARIOS := corridor_white_wall texture_mask_burst illumination_drop dual_failure_slam_and_range
DURATION  := 30

.PHONY: test gazebo-lint ablation figure stats headline paper-pdf clean

test:
	$(PY) -m pytest tests/

gazebo-lint:
	$(PY) -m pytest tests/test_gazebo_assets.py -v

$(RUNS_DIR):
	mkdir -p $(RUNS_DIR)

ablation: $(RUNS_DIR)
	$(PY) -m experiments.sim.cli \
	    --advisors rule_tree --scenarios $(SCENARIOS) \
	    --seeds $(SEEDS) --duration $(DURATION) \
	    --out $(RUNS_DIR)/acoustic_on.csv
	$(PY) -m experiments.sim.cli \
	    --advisors rule_tree --scenarios $(SCENARIOS) \
	    --seeds $(SEEDS) --duration $(DURATION) --no-acoustic \
	    --out $(RUNS_DIR)/acoustic_off.csv
	cat $(RUNS_DIR)/acoustic_on.csv > $(RUNS_DIR)/acoustic_combined.csv
	tail -n +2 $(RUNS_DIR)/acoustic_off.csv >> $(RUNS_DIR)/acoustic_combined.csv
	$(PY) -m experiments.sim.aggregate $(RUNS_DIR)/acoustic_combined.csv \
	    --out $(RUNS_DIR)/acoustic_summary.csv
	@echo "Wrote $(RUNS_DIR)/{acoustic_on,acoustic_off,acoustic_combined,acoustic_summary}.csv"

figure: $(FIG_DIR)/trajectory_acoustic_comparison.png

$(FIG_DIR)/trajectory_acoustic_comparison.png:
	$(PY) -m experiments.sim.visualize \
	    --scenario corridor_white_wall \
	    --seeds 0 1 2 3 4 5 6 7 \
	    --out $@

stats: $(RUNS_DIR)/acoustic_combined.csv
	$(PY) -m experiments.sim.paired_stats $(RUNS_DIR)/acoustic_combined.csv \
	    --out $(RUNS_DIR)/acoustic_paired_stats.csv
	@echo "--- paired Wilcoxon ---"
	@cat $(RUNS_DIR)/acoustic_paired_stats.csv

headline: test ablation figure stats
	@echo "=========================================="
	@echo "Headline result reproduced. Artifacts:"
	@echo "  $(RUNS_DIR)/acoustic_summary.csv"
	@echo "  $(RUNS_DIR)/acoustic_paired_stats.csv"
	@echo "  $(FIG_DIR)/trajectory_acoustic_comparison.png"
	@echo "=========================================="

paper-pdf:
	cd paper && $(MAKE)

clean:
	rm -rf $(RUNS_DIR)
	rm -f $(FIG_DIR)/trajectory_acoustic_comparison.png
	cd paper && $(MAKE) clean 2>/dev/null || true
