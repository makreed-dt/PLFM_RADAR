#!/bin/bash
# ===========================================================================
# FPGA Regression Test Runner for AERIS-10 Radar
# Runs all verified iverilog testbenches and reports pass/fail summary.
#
# Usage:  ./run_regression.sh [--quick]
#   --quick   Skip long-running integration tests (receiver golden, system TB)
#
# Exit code: 0 if all tests pass, 1 if any fail
# ===========================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

QUICK=0
if [[ "${1:-}" == "--quick" ]]; then
    QUICK=1
fi

PASS=0
FAIL=0
SKIP=0
ERRORS=""

# Colors (if terminal supports it)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

# ---------------------------------------------------------------------------
# Helper: compile and run a single testbench
#   run_test <name> <vvp_path> <iverilog_args...>
# ---------------------------------------------------------------------------
run_test() {
    local name="$1"
    local vvp="$2"
    shift 2
    local args=("$@")

    printf "  %-45s " "$name"

    # Compile
    if ! iverilog -g2001 -DSIMULATION -o "$vvp" "${args[@]}" 2>/tmp/iverilog_err_$$; then
        echo -e "${RED}COMPILE FAIL${NC}"
        ERRORS="$ERRORS\n  $name: compile error ($(head -1 /tmp/iverilog_err_$$))"
        FAIL=$((FAIL + 1))
        return
    fi

    # Run
    local output
    output=$(timeout 120 vvp "$vvp" 2>&1) || true

    # Count PASS/FAIL in output (testbenches use explicit [PASS]/[FAIL] markers)
    local test_pass test_fail
    test_pass=$(echo "$output" | grep -ci '\bPASS\b' || true)
    test_fail=$(echo "$output" | grep -ci '\bFAIL\b' || true)

    if [[ "$test_fail" -gt 0 ]]; then
        echo -e "${RED}FAIL${NC} (pass=$test_pass, fail=$test_fail)"
        ERRORS="$ERRORS\n  $name: $test_fail failure(s)"
        FAIL=$((FAIL + 1))
    elif [[ "$test_pass" -gt 0 ]]; then
        echo -e "${GREEN}PASS${NC} ($test_pass checks)"
        PASS=$((PASS + 1))
    else
        # No PASS/FAIL markers — check for clean completion
        if echo "$output" | grep -qi 'finish\|complete\|done'; then
            echo -e "${GREEN}PASS${NC} (completed)"
            PASS=$((PASS + 1))
        else
            echo -e "${YELLOW}UNKNOWN${NC} (no PASS/FAIL markers)"
            ERRORS="$ERRORS\n  $name: no pass/fail markers in output"
            FAIL=$((FAIL + 1))
        fi
    fi

    rm -f "$vvp"
}

# ===========================================================================
echo "============================================"
echo "  AERIS-10 FPGA Regression Test Suite"
echo "============================================"
echo ""
echo "Date: $(date)"
echo "iverilog: $(iverilog -V 2>&1 | head -1)"
echo ""

# ===========================================================================
# UNIT TESTS — Changed Modules (HIGH PRIORITY)
# ===========================================================================
echo "--- HIGH PRIORITY: Changed Modules ---"

run_test "CIC Decimator" \
    tb/tb_cic_reg.vvp \
    tb/tb_cic_decimator.v cic_decimator_4x_enhanced.v

run_test "Chirp Controller (BRAM)" \
    tb/tb_chirp_reg.vvp \
    tb/tb_chirp_controller.v plfm_chirp_controller.v

run_test "Chirp Contract" \
    tb/tb_chirp_ctr_reg.vvp \
    tb/tb_chirp_contract.v plfm_chirp_controller.v

run_test "Doppler Processor (DSP48)" \
    tb/tb_doppler_reg.vvp \
    tb/tb_doppler_cosim.v doppler_processor.v xfft_32.v fft_engine.v

echo ""

# ===========================================================================
# INTEGRATION TESTS
# ===========================================================================
echo "--- INTEGRATION TESTS ---"

run_test "DDC Chain (NCO→CIC→FIR)" \
    tb/tb_ddc_reg.vvp \
    tb/tb_ddc_cosim.v ddc_400m.v nco_400m_enhanced.v \
    cic_decimator_4x_enhanced.v fir_lowpass.v cdc_modules.v

if [[ "$QUICK" -eq 0 ]]; then
    # Golden generate
    run_test "Receiver (golden generate)" \
        tb/tb_rx_golden_reg.vvp \
        -DGOLDEN_GENERATE \
        tb/tb_radar_receiver_final.v radar_receiver_final.v \
        radar_mode_controller.v tb/ad9484_interface_400m_stub.v \
        ddc_400m.v nco_400m_enhanced.v cic_decimator_4x_enhanced.v \
        cdc_modules.v fir_lowpass.v ddc_input_interface.v \
        chirp_memory_loader_param.v latency_buffer.v \
        matched_filter_multi_segment.v matched_filter_processing_chain.v \
        range_bin_decimator.v doppler_processor.v xfft_32.v fft_engine.v

    # Golden compare
    run_test "Receiver (golden compare)" \
        tb/tb_rx_compare_reg.vvp \
        tb/tb_radar_receiver_final.v radar_receiver_final.v \
        radar_mode_controller.v tb/ad9484_interface_400m_stub.v \
        ddc_400m.v nco_400m_enhanced.v cic_decimator_4x_enhanced.v \
        cdc_modules.v fir_lowpass.v ddc_input_interface.v \
        chirp_memory_loader_param.v latency_buffer.v \
        matched_filter_multi_segment.v matched_filter_processing_chain.v \
        range_bin_decimator.v doppler_processor.v xfft_32.v fft_engine.v

    # Full system top
    run_test "System Top (radar_system_tb)" \
        tb/tb_system_reg.vvp \
        tb/radar_system_tb.v radar_system_top.v \
        radar_transmitter.v dac_interface_single.v plfm_chirp_controller.v \
        radar_receiver_final.v tb/ad9484_interface_400m_stub.v \
        ddc_400m.v nco_400m_enhanced.v cic_decimator_4x_enhanced.v \
        cdc_modules.v fir_lowpass.v ddc_input_interface.v \
        chirp_memory_loader_param.v latency_buffer.v \
        matched_filter_multi_segment.v matched_filter_processing_chain.v \
        range_bin_decimator.v doppler_processor.v xfft_32.v fft_engine.v \
        usb_data_interface.v edge_detector.v radar_mode_controller.v
else
    echo "  (skipped receiver golden + system top — use without --quick)"
    SKIP=$((SKIP + 3))
fi

echo ""

# ===========================================================================
# UNIT TESTS — Signal Processing
# ===========================================================================
echo "--- UNIT TESTS: Signal Processing ---"

run_test "FFT Engine" \
    tb/tb_fft_reg.vvp \
    tb/tb_fft_engine.v fft_engine.v

run_test "XFFT-32 Wrapper" \
    tb/tb_xfft_reg.vvp \
    tb/tb_xfft_32.v xfft_32.v fft_engine.v

run_test "NCO 400MHz" \
    tb/tb_nco_reg.vvp \
    tb/tb_nco_400m.v nco_400m_enhanced.v

run_test "FIR Lowpass" \
    tb/tb_fir_reg.vvp \
    tb/tb_fir_lowpass.v fir_lowpass.v

run_test "Matched Filter Chain" \
    tb/tb_mf_reg.vvp \
    tb/tb_matched_filter_processing_chain.v matched_filter_processing_chain.v \
    xfft_32.v fft_engine.v chirp_memory_loader_param.v

echo ""

# ===========================================================================
# UNIT TESTS — Infrastructure
# ===========================================================================
echo "--- UNIT TESTS: Infrastructure ---"

run_test "CDC Modules (3 variants)" \
    tb/tb_cdc_reg.vvp \
    tb/tb_cdc_modules.v cdc_modules.v

run_test "Edge Detector" \
    tb/tb_edge_reg.vvp \
    tb/tb_edge_detector.v edge_detector.v

run_test "USB Data Interface" \
    tb/tb_usb_reg.vvp \
    tb/tb_usb_data_interface.v usb_data_interface.v

run_test "Range Bin Decimator" \
    tb/tb_rbd_reg.vvp \
    tb/tb_range_bin_decimator.v range_bin_decimator.v

run_test "Radar Mode Controller" \
    tb/tb_rmc_reg.vvp \
    tb/tb_radar_mode_controller.v radar_mode_controller.v

echo ""

# ===========================================================================
# SUMMARY
# ===========================================================================
TOTAL=$((PASS + FAIL + SKIP))
echo "============================================"
echo "  RESULTS: $PASS passed, $FAIL failed, $SKIP skipped / $TOTAL total"
echo "============================================"

if [[ -n "$ERRORS" ]]; then
    echo ""
    echo "Failures:"
    echo -e "$ERRORS"
fi

echo ""

# Exit with error if any failures
if [[ "$FAIL" -gt 0 ]]; then
    exit 1
fi

exit 0
