# overclock_mt6768

Out-of-tree CPU/GPU overclock module for the MediaTek MT6768 kernel used by Xiaomi `moon`.

This version keeps the public control surface deliberately small:

- CPU: frequency only (Little + Big)
- GPU: frequency + VGPU
- GPU VSRAM: calculated automatically from VGPU using the Moon gpufreq rule
- read-only diagnostic tables/status remain available for validation

## Public control nodes

```text
/sys/module/overclock_mt6768/parameters/
├── cpu_ll_target_khz
├── cpu_b_target_khz
├── cpu_oc_apply
├── cpu_oc_result
├── gpu_target_khz
├── gpu_target_volt
├── gpu_oc_apply
└── gpu_oc_result
```

CPU voltage is intentionally not exposed as a user control. The CPU OC path applies the requested frequency while leaving the stock VPROC field unchanged. EEM/DVFS therefore remains responsible for CPU voltage selection.

GPU VSRAM is intentionally not exposed as a user control. When applying GPU OC, the module derives VSRAM automatically:

```text
if VGPU > 75000:
    VSRAM = VGPU + 10000
else:
    VSRAM = 85000
```

These values are in the same mV*100 units used by the vendor gpufreq code.

## Diagnostic nodes

The source may also expose read-only/internal validation nodes such as `gpu_opp_table`, `cpu_opp_table`, `cpu_pvt_table`, `cpu_sys_view`, `cpu_guard_hits`, and the topology nodes. They are not required as user controls and can be removed separately if a minimal release interface is desired.

## CPU path

Moon uses the vendor `mt-cpufreq` / hybrid CPU-DVFS path rather than the MT6789 cpufreq-hw LUT path. The module resolves the Moon CPU DVFS structures, validates the live top OPP, quiesces the cluster, patches CSRAM PVT index 0 with the requested frequency, and mirrors the frequency into the live cpufreq structures.

Only the frequency is changed by the public OC interface. The voltage field is left at its stock value.

## GPU path

The module patches the Moon gpufreq working table at index 0 and supplies the requested frequency and VGPU. Moon's gpufreq code owns the MFGPLL programming path, so this module does not expose or force a second manual PLL-write control.

## Apply examples

```sh
P=/sys/module/overclock_mt6768/parameters

# CPU Little frequency only
echo 1900000 > $P/cpu_ll_target_khz
echo 0 > $P/cpu_b_target_khz
echo 1 > $P/cpu_oc_apply
cat $P/cpu_oc_result

# CPU Big frequency only
echo 2100000 > $P/cpu_b_target_khz
echo 0 > $P/cpu_ll_target_khz
echo 1 > $P/cpu_oc_apply
cat $P/cpu_oc_result

# GPU frequency + VGPU; VSRAM follows automatically
echo 1100000 > $P/gpu_target_khz
echo 90000 > $P/gpu_target_volt
echo 1 > $P/gpu_oc_apply
cat $P/gpu_oc_result
```

Writing `0` to both CPU target-frequency nodes and applying restores the stock CPU top entries. Writing `0` to `gpu_target_khz` restores the stock GPU table.

## Important

This repository is source code, not a claim that every possible frequency/voltage combination is hardware-safe. Use the already-tested Moon limits and increase targets in small steps. CPU frequency and GPU frequency/voltage must be validated on the actual device.

The implementation is based on the `Mayuri-Chan/android_kernel_xiaomi_moon` `moon-u-oss` driver layout and does not import MT6789-specific CPU LUT assumptions.
