// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/kprobes.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/suspend.h>
#include <linux/string.h>
#include <linux/cpufreq.h>
#include <linux/bitfield.h>
#include <linux/pm_qos.h>
#include <linux/kthread.h>
#include <linux/energy_model.h>
#include <linux/mm.h>

static atomic_t oc_mt6768_suspended = ATOMIC_INIT(0);

static int oc_mt6768_pm_notifier(struct notifier_block *nb,
			       unsigned long action, void *data)
{
	switch (action) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
		atomic_set(&oc_mt6768_suspended, 1);
		break;
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
		atomic_set(&oc_mt6768_suspended, 0);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block oc_mt6768_pm_nb = {
	.notifier_call = oc_mt6768_pm_notifier,
};

static __nocfi int __kprobe_nop(struct kprobe *kp, struct pt_regs *regs)
{
	return 0;
}

static unsigned long resolve_ksym(const char *name)
{
	struct kprobe kp = {
		.symbol_name = name,
		.pre_handler = __kprobe_nop,
	};
	unsigned long addr;
	int ret = register_kprobe(&kp);

	if (ret < 0)
		return 0;
	addr = (unsigned long)kp.addr;
	unregister_kprobe(&kp);
	return addr;
}

enum gpufreq_posdiv {
	POSDIV_POWER_1  = 0,
	POSDIV_POWER_2  = 1,
	POSDIV_POWER_4  = 2,
	POSDIV_POWER_8  = 3,
	POSDIV_POWER_16 = 4,
};
enum gpufreq_target { TARGET_STACK = 0, TARGET_GPU = 1, TARGET_DEFAULT = 2 };

struct gpufreq_opp_info {
	unsigned int        freq;
	unsigned int        volt;
	unsigned int        vsram;
	enum gpufreq_posdiv posdiv;
	unsigned int        vaging;
	unsigned int        power;
};

typedef const struct gpufreq_opp_info *(*fn_get_wt_t)(enum gpufreq_target);
typedef int (*fn_get_opp_num_t)(enum gpufreq_target);

static fn_get_wt_t      sym_gpufreq_get_working_table;
static fn_get_opp_num_t sym_gpufreq_get_opp_num;

static fn_get_wt_t      sym_gpufreq_get_signed_table;

static struct gpufreq_opp_info **sym_ged_g_working_table;

typedef unsigned long (*fn_kallsyms_lookup_name_t)(const char *name);
static fn_kallsyms_lookup_name_t sym_kallsyms_lookup_name;

typedef bool (*fn_mtk_fh_set_rate_t)(const char *name, unsigned long dds, int postdiv);
/*
 * On moon `mtk_fh_set_rate` is NOT a function: drivers/clk/mediatek/clk-pll.c
 * defines `bool (*mtk_fh_set_rate)(const char *, unsigned long, int)` as an
 * EXPORTED FUNCTION-POINTER VARIABLE (NULL until fhctl probes, then set to
 * _mtk_fh_set_rate). kallsyms therefore gives the address of that variable;
 * calling it directly jumps into .data/.bss. Keep the address of the variable
 * and dereference it at call time.
 */
static fn_mtk_fh_set_rate_t *sym_mtk_fh_set_rate_pp;

/* Note: this used to also declare sym_cpufreq_stats_free_table/
 * sym_cpufreq_stats_create_table for a cpu_stats_refresh feature tied to
 * the old cpufreq-hw CPU approach. Removed along with that section — see
 * the big comment at the start of the CPU-OC block below for why.
 */

#define MFG_PLL_NAME "mfgpll"

#define APMIXED_PHYS     0x1000C000UL
#define APMIXED_SIZE     0x1000UL
/*
 * PORTED FROM MT6789 (original offset there was 0x26C).
 * MT6768's own gpufreq driver
 * (drivers/gpu/mediatek/gpufreq/v2/include/gpufreq_mt6768.h) defines
 * MFGPLL_CON1 as (g_apmixed_base + 0x24C) -- confirmed directly from the
 * moon kernel tree, not guessed. Do NOT reuse the MT6789 value (0x26C) on
 * MT6768; APMIXEDSYS hosts many unrelated PLLs and a wrong offset writes
 * to the wrong one.
 */
#define MFGPLL_CON1_OFF  0x24C

static void __iomem *g_apmixed_va;
#define MFGPLL_CON1  (g_apmixed_va + MFGPLL_CON1_OFF)

#define PLL_FIN        26U
#define DDS_SHIFT      14U
#define TO_MHZ_HEAD    100U
#define TO_MHZ_TAIL    10U
#define ROUNDING_VALUE 5U
#define POSDIV_SHIFT   24U

/*
 * UPDATED using real data from a tester's device (see PORTING_NOTES / README
 * porting-status table). `gpu_opp_table` dump confirmed moon's actual stock
 * GPU OPP table: 32 levels, from 299000 KHz up to a STOCK MAX of
 * 1000000 KHz @ ~87500 (mV*100) / vsram 97500. The old MT6789 ceiling
 * (1750000) was 75% above that stock max — reckless for a smaller/weaker
 * MT6768 GPU. The ceiling is now +30% over stock (1300000 KHz). It is only
 * a software guard, NOT a proven-safe number: raise the target in small
 * steps, after each step is confirmed stable on real hardware — do not jump
 * straight to the ceiling. Override at build time with
 * `make OC_DEFS="-DPOSDIV_2_MAX_FREQ=1400000U"` if you need more.
 *
 * IMPORTANT: this constant is only exercised on the WRITE path
 * (gpu_oc_apply, via write_pll_con1_safe/freq_to_posdiv_gpu). The
 * `gpu_opp_table` dump that already worked on the tester's device only
 * exercises the READ path (gpufreq_get_working_table) and does NOT prove
 * the write path / PLL register offset is correct in practice yet — that
 * still needs its own first real test, starting at/near stock frequency,
 * not at this ceiling.
 */
#ifndef POSDIV_2_MAX_FREQ
#define POSDIV_2_MAX_FREQ  1300000U
#endif
#define POSDIV_4_MIN_FREQ  375000U
#define VGPU_MAX_VOLT      95000U   /* VSRAM_MAX_VOLT 105000 - 10000 diff (gpufreq_mt6768.h) */
#define VGPU_MIN_VOLT      50000U
#define VSRAM_MAX_VOLT_LIM 105000U  /* VSRAM_MAX_VOLT in gpufreq_mt6768.h */
/*
 * Voltage bounds above (50000-100000, in the same mV*100-ish unit the dump
 * printed) already safely bracket the real dumped stock range
 * (75000-87500 volt, 85000-97500 vsram) with headroom on both sides, so
 * left unchanged — but still treat any value above stock's 87500/97500 as
 * unverified until tested.
 */

#define GPU_OPP_MAX 72

static unsigned int gpu_target_khz  = 0;
static unsigned int gpu_target_volt  = 0;
static int          gpu_oc_apply     = 0;
static char gpu_oc_result[128] = "not applied";
static char gpu_opp_table[4096] = "";

module_param(gpu_target_khz,  uint, 0644);
MODULE_PARM_DESC(gpu_target_khz,  "GPU target freq KHz (0=off, max 1300000 in this port; stock max on moon is 1000000)");
module_param(gpu_target_volt,  uint, 0644);
MODULE_PARM_DESC(gpu_target_volt,  "GPU target volt mV*100 (e.g. 90000=900mV, max 95000)");
module_param_string(gpu_oc_result, gpu_oc_result, sizeof(gpu_oc_result), 0444);
MODULE_PARM_DESC(gpu_oc_result, "GPU OC status (read-only)");

static DEFINE_MUTEX(oc_lock);

static bool                     g_gpu_patched[GPU_OPP_MAX];
static struct gpufreq_opp_info  g_gpu_orig[GPU_OPP_MAX];
static atomic_t                 g_last_commit_idx = ATOMIC_INIT(-1);
static unsigned int             g_last_pll_freq;
static bool                     g_gpu_oc_active;
static bool                     g_hw_mapped;

static struct workqueue_struct *g_oc_wq;
static struct delayed_work      g_gpu_pll_work;

static unsigned int calc_pcw(unsigned int freq_khz, enum gpufreq_posdiv posdiv)
{
	return (((freq_khz / TO_MHZ_HEAD * (1u << (unsigned)posdiv)) << DDS_SHIFT)
		/ PLL_FIN + ROUNDING_VALUE) / TO_MHZ_TAIL;
}

static enum gpufreq_posdiv freq_to_posdiv_gpu(unsigned int freq_khz)
{
	if (freq_khz > 950000)  return POSDIV_POWER_2;
	if (freq_khz > 475000)  return POSDIV_POWER_4;
	if (freq_khz > 237500)  return POSDIV_POWER_8;
	return POSDIV_POWER_16;
}

static __nocfi bool write_pll_con1_safe(void __iomem *reg, const char *pll_name,
				 unsigned int freq_old_khz, unsigned int freq_new_khz)
{
	enum gpufreq_posdiv target_posdiv;
	unsigned int pcw, pll;
	fn_mtk_fh_set_rate_t fh = sym_mtk_fh_set_rate_pp ? READ_ONCE(*sym_mtk_fh_set_rate_pp) : NULL;

	if (!reg || !fh)
		return false;

	target_posdiv = freq_to_posdiv_gpu(freq_new_khz);
	pcw = calc_pcw(freq_new_khz, target_posdiv);
	if (!pcw)
		return false;

	if (freq_new_khz > freq_old_khz) {

		if (!fh(pll_name, pcw, target_posdiv))
			return false;
		pll = (readl(reg) & 0xF8FFFFFFu) | ((unsigned)target_posdiv << POSDIV_SHIFT);
		writel(pll, reg);
		udelay(20);
	} else {

		pll = (readl(reg) & 0xF8FFFFFFu) | ((unsigned)target_posdiv << POSDIV_SHIFT);
		writel(pll, reg);
		udelay(20);
		if (!fh(pll_name, pcw, target_posdiv))
			return false;
	}
	return true;
}

static __nocfi void gpu_pll_work_fn(struct work_struct *work)
{
	struct gpufreq_opp_info *wt;
	unsigned int freq_old, freq_new;
	int idx;

	if (atomic_read(&oc_mt6768_suspended))
		return;
	if (!g_gpu_oc_active || !g_apmixed_va || !sym_gpufreq_get_working_table)
		return;

	idx = atomic_read(&g_last_commit_idx);
	if (idx < 0 || idx >= GPU_OPP_MAX || !g_gpu_patched[idx])
		return;

	wt = (struct gpufreq_opp_info *)sym_gpufreq_get_working_table(TARGET_GPU);
	if (!wt)
		return;

	freq_new = wt[idx].freq;
	freq_old = g_last_pll_freq ? g_last_pll_freq : g_gpu_orig[idx].freq;

	if (write_pll_con1_safe(MFGPLL_CON1, MFG_PLL_NAME, freq_old, freq_new))
		g_last_pll_freq = freq_new;
	else
		snprintf(gpu_oc_result, sizeof(gpu_oc_result),
			 "FAIL: PLL write refused (idx=%d) — OC not applied to PLL", idx);
}

static __nocfi int gpu_patch_one_index(struct gpufreq_opp_info *wt,
					struct gpufreq_opp_info *st,
					int n, int idx,
					unsigned int freq, unsigned int volt,
					unsigned int vsram)
{
	if (idx < 0 || idx >= n || idx >= GPU_OPP_MAX)
		return -EINVAL;

	if (!g_gpu_patched[idx]) {
		g_gpu_orig[idx] = wt[idx];
		g_gpu_patched[idx] = true;
	}

	wt[idx].freq   = freq;
	wt[idx].volt   = volt;
	wt[idx].vsram  = vsram;
	wt[idx].posdiv = freq_to_posdiv_gpu(freq);

	if (st) {
		st[idx].freq   = freq;
		st[idx].volt   = volt;
		st[idx].vsram  = vsram;
		st[idx].posdiv = freq_to_posdiv_gpu(freq);
	}
	return 0;
}

static __nocfi void ged_snapshot_resync(int idx, const struct gpufreq_opp_info *val)
{
	if (!sym_ged_g_working_table || !*sym_ged_g_working_table)
		return;

	(*sym_ged_g_working_table)[idx] = *val;
}

static __nocfi int gpu_patch_working_table(unsigned int freq, unsigned int volt,
			   unsigned int vsram)
{
	struct gpufreq_opp_info *wt, *st = NULL;
	int n, ret;

	if (!sym_gpufreq_get_working_table || !sym_gpufreq_get_opp_num) {
		snprintf(gpu_oc_result, sizeof(gpu_oc_result), "FAIL: GPU symbols not resolved");
		return -ENODEV;
	}

	n  = sym_gpufreq_get_opp_num(TARGET_GPU);
	wt = (struct gpufreq_opp_info *)sym_gpufreq_get_working_table(TARGET_GPU);
	if (sym_gpufreq_get_signed_table)
		st = (struct gpufreq_opp_info *)sym_gpufreq_get_signed_table(TARGET_GPU);

	if (!wt || n <= 0) {
		snprintf(gpu_oc_result, sizeof(gpu_oc_result), "FAIL: table NULL or opp_num=%d", n);
		return -ENODATA;
	}

	ret = gpu_patch_one_index(wt, st, n, 0, freq, volt, vsram);
	if (ret) {
		snprintf(gpu_oc_result, sizeof(gpu_oc_result), "FAIL: idx0 patch rejected (%d)", ret);
		return ret;
	}
	ged_snapshot_resync(0, &wt[0]);
	snprintf(gpu_oc_result, sizeof(gpu_oc_result),
		 "OK: idx0 freq=%u volt=%u vsram=%u (n=%d, signed=%s, ged_sync=%s)",
		 freq, volt, vsram, n, st ? "yes" : "no",
		 (sym_ged_g_working_table && *sym_ged_g_working_table) ? "yes" : "no");
	return 0;
}

static __nocfi void gpu_restore_working_table(void)
{
	struct gpufreq_opp_info *wt, *st = NULL;
	int i, n;

	if (!sym_gpufreq_get_working_table || !sym_gpufreq_get_opp_num)
		return;
	wt = (struct gpufreq_opp_info *)sym_gpufreq_get_working_table(TARGET_GPU);
	if (!wt)
		return;
	if (sym_gpufreq_get_signed_table)
		st = (struct gpufreq_opp_info *)sym_gpufreq_get_signed_table(TARGET_GPU);
	n = sym_gpufreq_get_opp_num(TARGET_GPU);

	for (i = 0; i < GPU_OPP_MAX && i < n; i++) {
		if (!g_gpu_patched[i])
			continue;
		wt[i] = g_gpu_orig[i];
		if (st)
			st[i] = g_gpu_orig[i];
		ged_snapshot_resync(i, &g_gpu_orig[i]);
		g_gpu_patched[i] = false;
	}
	atomic_set(&g_last_commit_idx, -1);
	g_last_pll_freq = 0;
}

static __nocfi void update_gpu_opp_table(void)
{
	struct gpufreq_opp_info *wt;
	int i, n, off = 0;

	if (!sym_gpufreq_get_working_table || !sym_gpufreq_get_opp_num) {
		snprintf(gpu_opp_table, sizeof(gpu_opp_table), "symbols not ready");
		return;
	}
	n  = sym_gpufreq_get_opp_num(TARGET_GPU);
	wt = (struct gpufreq_opp_info *)sym_gpufreq_get_working_table(TARGET_GPU);
	if (!wt || n <= 0) { snprintf(gpu_opp_table, sizeof(gpu_opp_table), "unavailable"); return; }

	off = snprintf(gpu_opp_table, sizeof(gpu_opp_table), "GPU OPP (n=%d):\n", n);
	for (i = 0; i < n && off < (int)sizeof(gpu_opp_table) - 60; i++)
		off += snprintf(gpu_opp_table + off, sizeof(gpu_opp_table) - off,
			"[%2d]%s %7u KHz %7u mV*100 vsram=%7u\n",
			i, (i < GPU_OPP_MAX && g_gpu_patched[i]) ? "*" : " ",
			wt[i].freq, wt[i].volt, wt[i].vsram);
}

static int gpu_opp_table_get(char *buf, const struct kernel_param *kp)
{
	mutex_lock(&oc_lock);
	update_gpu_opp_table();
	mutex_unlock(&oc_lock);
	return scnprintf(buf, PAGE_SIZE, "%s", gpu_opp_table);
}
static const struct kernel_param_ops gpu_opp_table_ops = { .get = gpu_opp_table_get };
module_param_cb(gpu_opp_table, &gpu_opp_table_ops, NULL, 0444);
MODULE_PARM_DESC(gpu_opp_table, "READ-ONLY: live GPU OPP table dump (freq/volt/vsram), recomputed on every read");

static struct kretprobe krp_gpufreq_commit;

struct gpu_commit_args {
	enum gpufreq_target target;
	int                 oppidx;
};

static __nocfi int krp_gpufreq_commit_entry(struct kretprobe_instance *ri,
					    struct pt_regs *regs)
{
	struct gpu_commit_args *a = (struct gpu_commit_args *)ri->data;

	a->target = (enum gpufreq_target)(regs->regs[0] & 0xFFFFFFFFu);
	a->oppidx = (int)(regs->regs[1] & 0xFFFFFFFFu);
	return 0;
}

static __nocfi int krp_gpufreq_commit_ret(struct kretprobe_instance *ri,
					  struct pt_regs *regs)
{
	struct gpu_commit_args *a = (struct gpu_commit_args *)ri->data;
	if (atomic_read(&oc_mt6768_suspended))
		return 0;
	return 0;
}

static int lazy_map_hw(void)
{
	if (g_hw_mapped)
		return 0;

	g_apmixed_va = ioremap(APMIXED_PHYS, APMIXED_SIZE);
	if (!g_apmixed_va)
		pr_warn("oc_mt6768: apmixed ioremap failed -> GPU PLL write disabled\n");

	g_hw_mapped = true;
	return g_apmixed_va ? 0 : -ENODEV;
}

static int gpu_oc_apply_set(const char *val, const struct kernel_param *kpp)
{
	int v, ret;
	unsigned int auto_vsram;

	ret = kstrtoint(val, 0, &v);
	if (ret || v != 1)
		return ret;

	if (atomic_read(&oc_mt6768_suspended)) {
		snprintf(gpu_oc_result, sizeof(gpu_oc_result), "FAIL: device suspending, refused");
		gpu_oc_apply = 0;
		return 0;
	}

	mutex_lock(&oc_lock);

	if (lazy_map_hw() < 0) {
		snprintf(gpu_oc_result, sizeof(gpu_oc_result), "FAIL: hardware mapping failed");
		goto out;
	}

	if (gpu_target_khz == 0) {
		cancel_delayed_work_sync(&g_gpu_pll_work);
		gpu_restore_working_table();
		g_gpu_oc_active = false;
		snprintf(gpu_oc_result, sizeof(gpu_oc_result), "OK: GPU OC disabled, stock restored");
		goto out;
	}

	if (gpu_target_khz < POSDIV_4_MIN_FREQ || gpu_target_khz > POSDIV_2_MAX_FREQ) {
		snprintf(gpu_oc_result, sizeof(gpu_oc_result),
			 "FAIL: freq %u out of [%u, %u] KHz",
			 gpu_target_khz, POSDIV_4_MIN_FREQ, POSDIV_2_MAX_FREQ);
		goto out;
	}
	if (gpu_target_volt < VGPU_MIN_VOLT || gpu_target_volt > VGPU_MAX_VOLT) {
		snprintf(gpu_oc_result, sizeof(gpu_oc_result),
			 "FAIL: volt %u out of [%u, %u] mV*100",
			 gpu_target_volt, VGPU_MIN_VOLT, VGPU_MAX_VOLT);
		goto out;
	}
	/* Moon gpufreq rule: Vgpu > 75000 -> Vsram = Vgpu + 10000, else fixed 85000.
	 * VSRAM is intentionally not a public control; it follows VGPU automatically. */
	auto_vsram = (gpu_target_volt > 75000U) ? gpu_target_volt + 10000U : 85000U;
	if (auto_vsram < VGPU_MIN_VOLT || auto_vsram > VSRAM_MAX_VOLT_LIM) {
		snprintf(gpu_oc_result, sizeof(gpu_oc_result),
			 "FAIL: auto VSRAM %u out of [%u, %u] mV*100",
			 auto_vsram, VGPU_MIN_VOLT, VSRAM_MAX_VOLT_LIM);
		goto out;
	}

	ret = gpu_patch_working_table(gpu_target_khz, gpu_target_volt, auto_vsram);
	if (ret)
		goto out;

	g_gpu_oc_active = true;
	update_gpu_opp_table();

out:
	gpu_oc_apply = 0;
	mutex_unlock(&oc_lock);
	return 0;
}
static int gpu_oc_apply_get(char *buf, const struct kernel_param *kpp)
{
	return scnprintf(buf, PAGE_SIZE, "0\n");
}
static const struct kernel_param_ops gpu_oc_apply_ops = {
	.set = gpu_oc_apply_set,
	.get = gpu_oc_apply_get,
};
module_param_cb(gpu_oc_apply, &gpu_oc_apply_ops, &gpu_oc_apply, 0644);
MODULE_PARM_DESC(gpu_oc_apply, "Write 1 to apply gpu_target_khz/volt/vsram to OPP idx0");

/*
 * ============================================================================
 * CPU OC — REWRITTEN FOR MT6768's REAL DRIVER, NOT PORTED FROM MT6789
 * ============================================================================
 *
 * A tester loaded the MT6789-derived version of this section on a real
 * moon device and it failed outright:
 *
 *   cpu0..cpu7: no driver_data/freq_table (driver != mtk-cpufreq-hw?)
 *
 * Root cause, confirmed directly from the moon kernel tree, not guessed:
 * moon's CPU nodes declare `compatible = "mediatek,mt-cpufreq"`
 * (arch/arm64/boot/dts/mediatek/mt6768.dts), a completely different,
 * older-generation MediaTek CPU DVFS driver
 * (drivers/misc/mediatek/cpufreq_v2/ in that tree) — NOT the generic
 * `mediatek-cpufreq-hw.c` / ARM "cpufreq-hw" MMIO-LUT driver the MT6789
 * module was written against. Different driver, different data layout,
 * different public API. The old offset-style fix that worked for the GPU
 * side does not apply here; this needed a different implementation, not a
 * different constant.
 *
 * This driver's per-cluster live state lives in an exported data symbol:
 *
 *   extern struct mt_cpu_dvfs cpu_dvfs[NR_MT_CPU_DVFS];
 *   (drivers/misc/mediatek/cpufreq_v2/inc/mtk_cpufreq_struct.h,
 *    drivers/misc/mediatek/cpufreq_v2/src/mt6768/mtk_cpufreq_config.h)
 *
 * with cluster indices (confirmed from mtk_cpufreq_config.h):
 *   0 = MT_CPU_DVFS_LL  (little cluster — cpu0-5 per moon's mt6768.dts cpu-map)
 *   1 = MT_CPU_DVFS_L   (big cluster    — cpu6-7 per moon's mt6768.dts cpu-map)
 *   2 = MT_CPU_DVFS_CCI (interconnect, not a CPU cluster — left alone here)
 *
 * struct mt_cpu_dvfs holds `struct mt_cpu_freq_info *opp_tbl` (an array of
 * {cpufreq_khz, cpufreq_volt} pairs) and `int nr_opp_tbl` (its length) —
 * this is the actual table the driver reads during DVFS. Same idea as the
 * GPU side's g_working_table patch: resolve the symbol via kallsyms, then
 * read/write that live table directly. This has NOT been tested on real
 * hardware yet — treat the first load the same cautious way the GPU side
 * was first tested (dump before touching anything).
 *
 * Deliberately NOT implemented in this version, to keep the first real
 * test narrow:
 *   - No watchdog/guardian thread re-asserting the OC value (the old
 *     cpufreq-hw section had one; this one does not yet).
 *   - No attempt to add a new OPP index beyond the stock table — only the
 *     existing top index's khz/volt are overwritten. Growing the array
 *     would need to know its real allocated capacity, which isn't exposed
 *     safely.
 *   - CCI (cluster id 2) is left completely untouched.
 * Treat this as a v1: dump-and-bump-top-index, nothing more.
 */

/* Mirrors struct mt_cpu_freq_info from mtk_cpufreq_struct.h — 2 plain
 * unsigned ints, so there's nothing to get wrong here.
 */
struct mt_cpu_freq_info_mirror {
	unsigned int cpufreq_khz;
	unsigned int cpufreq_volt;
};

/* Mirrors struct mt_cpu_dvfs from mtk_cpufreq_struct.h, field-for-field,
 * in the same order, so the offsets line up with the real compiled struct
 * in moon's kernel. Enum fields are mirrored as `int` (their real
 * underlying type in a normal, non -fshort-enums kernel build), and
 * pointer fields we don't touch are kept as `void *` — we never
 * dereference those, just need the struct's total layout to match so the
 * fields we DO use (opp_tbl, nr_opp_tbl, idx_opp_tbl) land at the right
 * byte offset.
 */
struct mt_cpu_dvfs_mirror {
	const char *name;
	int id;
	unsigned int cpu_id;
	unsigned int cpu_level;
	void *mt_policy;
	int Vproc_buck_id;
	int Vsram_buck_id;
	int Pll_id;
	void *freq_tbl;
	struct mt_cpu_freq_info_mirror *opp_tbl;
	int nr_opp_tbl;
	int idx_opp_tbl;
	int idx_opp_ppm_base;
	int idx_opp_ppm_limit;
	int armpll_is_available;
	int idx_normal_max_opp;
	void *freq_tbl_for_cpufreq;
	bool dvfs_disable_by_suspend;
	bool dvfs_disable_by_procfs;
	unsigned int turbo_mode;
};

#define MT_CPU_DVFS_LL   0
#define MT_CPU_DVFS_L    1
#define NR_MT_CPU_DVFS   3
/* Loose upper bound just to reject an obviously-wrong nr_opp_tbl read (struct
 * mismatch, symbol resolved to the wrong address, etc.) — not a real limit
 * from the driver, which defines NR_FREQ=16 per cluster.
 */
#define NR_FREQ_SANITY   32

/* Informational only in this version — the new cluster_id-based API
 * doesn't need a "representative CPU#" the way the old cpufreq-hw
 * approach did. Left here, read-only, as a record of the topology
 * confirmed from moon's own arch/arm64/boot/dts/mediatek/mt6768.dts
 * cpu-map (cluster0 = cpu0-5, cluster1 = cpu6-7).
 */
static unsigned int cpu_ll_first_cpu = 0;
static unsigned int cpu_b_first_cpu  = 6;
module_param(cpu_ll_first_cpu, uint, 0444);
module_param(cpu_b_first_cpu, uint, 0444);
MODULE_PARM_DESC(cpu_ll_first_cpu, "First CPU# in the little cluster (informational, from moon's cpu-map)");
MODULE_PARM_DESC(cpu_b_first_cpu, "First CPU# in the big cluster (informational, from moon's cpu-map)");

/* Real voltage rails sourced from moon's own
 * drivers/misc/mediatek/cpufreq_v2/src/mt6768/mtk_cpufreq_platform.h —
 * not carried over from MT6789. Same mV*100-style unit the GPU dump uses.
 */
#define CPU_MIN_VPROC_VOLT   60000U
#define CPU_MAX_VPROC_VOLT   112000U

/* No hardware-sourced absolute frequency ceiling exists for this driver
 * (unlike the voltage rails above) — the safety mechanism here is purely
 * "percent above whatever the live top index currently reads", checked at
 * apply time against each cluster's own current value. See
 * MAX_OC_PERCENT_OVER_STOCK below.
 */
#ifndef MAX_OC_PERCENT_OVER_STOCK
#define MAX_OC_PERCENT_OVER_STOCK  20
#endif

/*
 * WHY NOT kallsyms_lookup_name("cpu_dvfs"):
 * moon builds several vendor modules, and `cpu_dvfs` is NOT a unique name:
 *   - CPU_DVFS.ko          : `struct mt_cpu_dvfs cpu_dvfs[NR_MT_CPU_DVFS]` (what we want)
 *   - mtk_fpsgo.ko (fbt_cpu.c): `static struct fbt_cpu_dvfs_info *cpu_dvfs;` (a lone pointer)
 * kallsyms_lookup_name() returns the first module that matches, so it can hand
 * back the fpsgo pointer variable; reading it as an array of mt_cpu_dvfs gives
 * exactly the garbage seen on-device (nr_opp_tbl=1317585370). Instead resolve
 * the UNIQUE function id_to_cpu_dvfs() (mtk_cpufreq_main.c) and let the driver
 * itself return &cpu_dvfs[id]. Every result is checked with c->id == id.
 */
typedef struct mt_cpu_dvfs_mirror *(*fn_id_to_cpu_dvfs_t)(int id);
static fn_id_to_cpu_dvfs_t sym_id_to_cpu_dvfs;

/* Address of CPU_DVFS.ko's global `u32 *recordRef` (mtk_cpufreq_hybrid.c):
 * the ioremap'd CSRAM PVT table (phys CSRAM_BASE 0x00110C00 + 0x10) that the
 * MCUPM/hybrid DVFS hardware actually uses under HYBRID_CPU_DVFS. */
static u32 **sym_recordRef_pp;
typedef int (*fn_cur_idx_t)(int cluster_id);
static fn_cur_idx_t sym_cpuhvfs_get_cur_idx;	/* int cpuhvfs_get_cur_dvfs_freq_idx(int) in mtk_cpufreq_hybrid.c */

static __nocfi struct mt_cpu_dvfs_mirror *cluster_get(int cluster_id)
{
	struct mt_cpu_dvfs_mirror *c;

	if (!sym_id_to_cpu_dvfs)
		return NULL;
	c = sym_id_to_cpu_dvfs(cluster_id);
	if (!c || (int)c->id != cluster_id)
		return NULL;
	return c;
}

static unsigned int cpu_ll_target_khz = 0;
static unsigned int cpu_b_target_khz  = 0;
static int          cpu_oc_apply       = 0;
module_param(cpu_ll_target_khz, uint, 0644);
module_param(cpu_b_target_khz,  uint, 0644);
MODULE_PARM_DESC(cpu_ll_target_khz, "Little cluster idx0 target KHz, multiple of 1000, stock..+20%; 0 restores stock");
MODULE_PARM_DESC(cpu_b_target_khz,  "Big cluster idx0 target KHz, multiple of 1000, stock..+20%; 0 restores stock");

/* Saved stock top-index values so exit() can put them back. */
/* Stock CSRAM idx0 word per cluster (0=little, 1=big), saved before the first write. */
static u32  g_orig_w0[2];
static bool g_have_w0[2];

static char cpu_oc_result[512] = "not applied yet";
static size_t cpu_res_off;
#define CPU_RES(...) (cpu_res_off += scnprintf(cpu_oc_result + cpu_res_off, sizeof(cpu_oc_result) - cpu_res_off, __VA_ARGS__))
static int cpu_oc_result_get(char *buf, const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", cpu_oc_result);
}
static const struct kernel_param_ops cpu_oc_result_ops = { .get = cpu_oc_result_get };
module_param_cb(cpu_oc_result, &cpu_oc_result_ops, NULL, 0444);

/* Dump one cluster's real, live OPP table straight out of cpu_dvfs[]. */
static __nocfi void dump_one_cluster(int cluster_id, const char *label, char *buf,
			      size_t *off, size_t bufsize)
{
	struct mt_cpu_dvfs_mirror *c;
	int i;

	if (!sym_id_to_cpu_dvfs) {
		*off += scnprintf(buf + *off, bufsize - *off,
				   "%s: id_to_cpu_dvfs symbol not resolved (CPU_DVFS.ko loaded?)\n", label);
		return;
	}
	c = cluster_get(cluster_id);
	if (!c) {
		*off += scnprintf(buf + *off, bufsize - *off,
				   "%s: id_to_cpu_dvfs(%d) returned NULL or wrong id\n", label, cluster_id);
		return;
	}

	if (!c->opp_tbl || c->nr_opp_tbl <= 0 || c->nr_opp_tbl > NR_FREQ_SANITY) {
		*off += scnprintf(buf + *off, bufsize - *off,
				   "%s: opp_tbl=%p nr_opp_tbl=%d — looks invalid, struct mirror may not match this build\n",
				   label, c->opp_tbl, c->nr_opp_tbl);
		return;
	}

	*off += scnprintf(buf + *off, bufsize - *off,
			   "%s (id=%d, cpu_level=%u, nr_opp_tbl=%d, cur_idx=%d):\n",
			   label, c->id, c->cpu_level, c->nr_opp_tbl, c->idx_opp_tbl);
	for (i = 0; i < c->nr_opp_tbl && *off < bufsize - 64; i++) {
		*off += scnprintf(buf + *off, bufsize - *off,
				   "  [%2d]%s %7u KHz  volt=%u\n",
				   i, (i == c->idx_opp_tbl) ? " *" : "  ",
				   c->opp_tbl[i].cpufreq_khz,
				   c->opp_tbl[i].cpufreq_volt);
	}
}

static char cpu_opp_table_buf[4096];
static int cpu_opp_table_get(char *buf, const struct kernel_param *kp)
{
	size_t off = 0;

	dump_one_cluster(MT_CPU_DVFS_LL, "little", cpu_opp_table_buf, &off, sizeof(cpu_opp_table_buf));
	dump_one_cluster(MT_CPU_DVFS_L,  "big",    cpu_opp_table_buf, &off, sizeof(cpu_opp_table_buf));
	return scnprintf(buf, PAGE_SIZE, "%s", cpu_opp_table_buf);
}
static const struct kernel_param_ops cpu_opp_table_ops = { .get = cpu_opp_table_get };
module_param_cb(cpu_opp_table, &cpu_opp_table_ops, NULL, 0444);
MODULE_PARM_DESC(cpu_opp_table, "Read-only dump of both clusters' live OPP tables from cpu_dvfs[]");

/* CSRAM PVT table base (`recordRef`), or NULL when it is not safe to touch. */
static __nocfi u32 *pvt_ref(void)
{
	u32 *ref;

	if (!sym_recordRef_pp)
		return NULL;
	ref = READ_ONCE(*sym_recordRef_pp);
	if (!ref || !is_vmalloc_addr(ref))
		return NULL;
	return ref;
}

/* Same pieces the working MT6789/rock module uses to be visible to the system,
 * adapted to moon's driver:
 *  - quiesce_off_idx0(): cap the cluster to freq_table[1] with a temporary
 *    FREQ_QOS_MAX request and wait until the hardware current index leaves idx0
 *    (CSRAM cur-idx via the driver's own cpuhvfs_get_cur_dvfs_freq_idx()), so the
 *    CSRAM word is never rewritten while the cluster runs on it.
 *  - freq_tbl_for_cpufreq[0] + policy->cpuinfo.max_freq (+ the user FREQ_QOS max
 *    request, instead of rock's polling guardian thread): moon's
 *    _mt_cpufreq_target() maps freq -> index over that table.
 *  - Energy Model top perf-state frequency (thermal cooling / EAS read it).
 *  - cpufreq_stats top bucket (time_in_state) renamed old idx0 -> new idx0 via
 *    stats_relabel().
 * Raising order: table -> cpuinfo -> QoS. Lowering order: QoS -> table -> cpuinfo.
 */
static __nocfi int quiesce_off_idx0(struct cpufreq_policy *pol, int cluster_id,
				    struct freq_qos_request *req)
{
	unsigned int waited_us = 0;
	int ret;

	if (!sym_cpuhvfs_get_cur_idx)
		return -ENODEV;
	if (!pol->freq_table || pol->freq_table[1].frequency == CPUFREQ_TABLE_END)
		return -ENODATA;

	ret = freq_qos_add_request(&pol->constraints, req, FREQ_QOS_MAX,
				   pol->freq_table[1].frequency);
	if (ret < 0)
		return ret;

	while (waited_us < 50000U) {
		if (sym_cpuhvfs_get_cur_idx(cluster_id) != 0)
			return 0;
		usleep_range(500, 1000);
		waited_us += 500;
	}
	freq_qos_remove_request(req);
	return -ETIMEDOUT;
}

static int patch_em_top(unsigned int cpu, unsigned int khz)
{
	struct em_perf_domain *pd = em_cpu_get(cpu);
	struct em_perf_state *top;

	if (!pd || !pd->table || !pd->nr_perf_states)
		return -ENODEV;
	top = &pd->table[pd->nr_perf_states - 1];
	top->frequency = khz;
	return 0;
}

/*
 * time_in_state fix.
 *
 * cpufreq_stats (drivers/cpufreq/cpufreq_stats.c) builds its own private copy of
 * the frequency list ONCE, in cpufreq_stats_create_table() at policy init:
 *   stats->freq_table[] = copy of policy->freq_table frequencies (stock idx0!)
 * Patching policy->freq_table[0] afterwards does not touch that copy. Then
 * cpufreq_stats_record_transition() looks the new frequency up in the copy, gets
 * -1 for the OC frequency and silently returns, so the OC state never gets a
 * bucket (and its runtime is credited to whatever bucket was last).
 *
 * The MT6789 module fixed this with cpufreq_stats_free_table()+create_table().
 * That frees policy->stats while cpufreq_stats_record_transition() (which does
 * NOT take policy->rwsem) can still be dereferencing it, and it holds
 * policy->rwsem across sysfs_remove_group(), which drains readers of
 * time_in_state that are themselves waiting on policy->rwsem. Here we only
 * rewrite the one u32 in the existing table instead: no alloc/free, no sysfs
 * churn, no lifetime race.
 *
 * Mirror of struct cpufreq_stats from android12-5.10 (private to
 * cpufreq_stats.c). The layout is validated at runtime before any write.
 */
struct cpufreq_stats_mirror {
	unsigned int total_trans;
	unsigned long long last_time;
	unsigned int max_state;
	unsigned int state_num;
	unsigned int last_index;
	u64 *time_in_state;
	unsigned int *freq_table;
	unsigned int *trans_table;
	unsigned int reset_pending;
	unsigned long long reset_time;
};

/* policy->stats as a validated mirror, or NULL if the layout does not match. */
static struct cpufreq_stats_mirror *stats_get(struct cpufreq_policy *pol)
{
	struct cpufreq_stats_mirror *s;

	if (!pol || !pol->stats)
		return NULL;
	s = (struct cpufreq_stats_mirror *)pol->stats;

	/* freq_table lives right behind time_in_state in one allocation
	 * (cpufreq_stats_create_table); if that does not hold, the mirror is wrong. */
	if (!s->max_state || s->max_state > NR_FREQ_SANITY ||
	    s->state_num > s->max_state || !s->time_in_state ||
	    s->freq_table != (unsigned int *)(s->time_in_state + s->max_state))
		return NULL;
	return s;
}

/* Rename the stats bucket old_khz -> new_khz. The bucket's accumulated time
 * belonged to the OLD frequency, so it is zeroed rather than relabelled.
 * Returns 0 on success, negative errno (nothing written) otherwise.
 */
static int stats_relabel(struct cpufreq_policy *pol, unsigned int old_khz,
			 unsigned int new_khz)
{
	struct cpufreq_stats_mirror *s;
	int hit = -1;
	unsigned int i;

	if (!pol || !pol->stats)
		return -ENODEV;
	s = stats_get(pol);
	if (!s)
		return -EINVAL;

	for (i = 0; i < s->max_state; i++) {
		if (s->freq_table[i] == new_khz)
			return (new_khz == old_khz) ? 0 : -EEXIST;
		if (s->freq_table[i] == old_khz)
			hit = i;
	}
	if (hit < 0)
		return -ENOENT;

	WRITE_ONCE(s->time_in_state[hit], 0);
	WRITE_ONCE(s->freq_table[hit], new_khz);
	return 0;
}

static __nocfi void sys_expose(struct cpufreq_policy *pol, struct mt_cpu_dvfs_mirror *c,
			       unsigned int old_khz, unsigned int new_khz, const char *label)
{
	struct cpufreq_frequency_table *tbl = (struct cpufreq_frequency_table *)c->freq_tbl_for_cpufreq;
	bool raising = new_khz > old_khz;
	int req_val = (pol && pol->max_freq_req) ? pol->max_freq_req->pnode.prio : 0;

	if (new_khz != old_khz) {
		/* Cluster is quiesced off idx0 here, so no transition to/from the
		 * top OPP can be recorded while the bucket is renamed. */
		if (pol) {
			int sret = stats_relabel(pol, old_khz, new_khz);

			if (sret)
				CPU_RES("%s: time_in_state bucket %u->%u not renamed (%d)\n",
					label, old_khz, new_khz, sret);
		}

		if (pol && !raising && pol->max_freq_req && req_val == (int)old_khz)
			freq_qos_update_request(pol->max_freq_req, new_khz);

		if (tbl && tbl[0].frequency != CPUFREQ_TABLE_END && tbl[0].frequency != CPUFREQ_ENTRY_INVALID)
			tbl[0].frequency = new_khz;
		/* scaling_available_frequencies / cpufreq_frequency_table_target() read
		 * policy->freq_table. It is normally the same array as
		 * freq_tbl_for_cpufreq, but that is an assumption about the driver:
		 * if it is a separate copy, patch it as well. */
		if (pol && pol->freq_table && pol->freq_table != tbl &&
		    pol->freq_table[0].frequency == old_khz)
			pol->freq_table[0].frequency = new_khz;
		if (pol) {
			down_write(&pol->rwsem);
			pol->cpuinfo.max_freq = new_khz;
			up_write(&pol->rwsem);
		}

		if (pol && raising && pol->max_freq_req) {
			if (req_val == (int)old_khz)
				freq_qos_update_request(pol->max_freq_req, new_khz);
			else
				CPU_RES("%s: user QoS max is %d, policy->max not raised\n", label, req_val);
		}
	}
	if (patch_em_top(c->cpu_id, new_khz))
		CPU_RES("%s: no EM perf domain for cpu%u (thermal cooling may still pin stock max)\n", label, c->cpu_id);
}

/* Patch idx0 (highest OPP) of one cluster in the CSRAM PVT table that the
 * hybrid DVFS hardware uses, and mirror khz/volt into the RAM opp_tbl[0] so
 * the driver's own index search stays consistent. Word layout (verified by the
 * on-device cpu_pvt_table dump, ram=match on all 16 entries):
 *   word[cluster*36 + 0] = (vproc_raw[11:0] << 16) | freq_MHz[15:0]
 * posdiv/clkdiv words are NOT touched (idx0 is posdiv=1 clkdiv=1 on both
 * clusters, so VCO == target freq; only freqs up to 2000 MHz are seen in the
 * stock tables for posdiv=1, hence the small % cap).
 * Whether MCUPM re-reads this word after init is firmware behaviour: if the
 * clock does not change, that is the answer.
 */
static __nocfi int apply_one_cluster(int cluster_id, const char *label,
				     unsigned int target_khz, unsigned int target_volt)
{
	struct mt_cpu_dvfs_mirror *c = cluster_get(cluster_id);
	u32 *ref = pvt_ref();
	void __iomem *w0p;
	u32 w0;
	unsigned int orig_mhz, orig_raw, mhz, raw, cap_khz, prev_khz;
	struct cpufreq_policy *pol;
	struct freq_qos_request qreq;
	int qret;

	if (cluster_id < 0 || cluster_id > 1)
		return -EINVAL;
	if (!c || !ref) {
		CPU_RES("%s: id_to_cpu_dvfs/recordRef unresolved, nothing applied\n", label);
		return -ENODEV;
	}
	if (!c->opp_tbl || c->nr_opp_tbl <= 0 || c->nr_opp_tbl > NR_FREQ_SANITY) {
		CPU_RES("%s: RAM opp_tbl invalid, refusing\n", label);
		return -EINVAL;
	}

	w0p = (void __iomem *)(ref + cluster_id * 36);
	w0 = readl(w0p);
	if (!g_have_w0[cluster_id]) {
		if ((w0 & 0xFFFF) * 1000U != c->opp_tbl[0].cpufreq_khz) {
			CPU_RES("%s: CSRAM idx0 (%u MHz) != RAM idx0 (%u kHz), layout assumption wrong, refusing\n",
				label, w0 & 0xFFFF, c->opp_tbl[0].cpufreq_khz);
			return -EINVAL;
		}
		g_orig_w0[cluster_id] = w0;
		g_have_w0[cluster_id] = true;
	}
	orig_mhz = g_orig_w0[cluster_id] & 0xFFFF;
	orig_raw = (g_orig_w0[cluster_id] >> 16) & 0xFFF;

	mhz = w0 & 0xFFFF;
	if (target_khz) {
		if (target_khz % 1000U) {
			CPU_RES("%s: target_khz %u must be a multiple of 1000 (table is in MHz)\n", label, target_khz);
			return -EINVAL;
		}
		cap_khz = orig_mhz * 1000U + (orig_mhz * 1000U * MAX_OC_PERCENT_OVER_STOCK) / 100U;
		if (target_khz < orig_mhz * 1000U || target_khz > cap_khz) {
			CPU_RES("%s: target_khz %u outside [stock %u, +%d%% cap %u], refusing (0 restores stock)\n",
				label, target_khz, orig_mhz * 1000U, MAX_OC_PERCENT_OVER_STOCK, cap_khz);
			return -EINVAL;
		}
		mhz = target_khz / 1000U;
	}

	raw = (w0 >> 16) & 0xFFF;
	if (target_volt) {
		if (target_volt < CPU_MIN_VPROC_VOLT || target_volt > CPU_MAX_VPROC_VOLT) {
			CPU_RES("%s: target_volt %u out of [%u,%u], refusing\n",
				label, target_volt, CPU_MIN_VPROC_VOLT, CPU_MAX_VPROC_VOLT);
			return -EINVAL;
		}
		raw = (target_volt - 50000U) / 625U;	/* floor: never above the request */
		if (raw < orig_raw) {
			CPU_RES("%s: volt below stock idx0 (%u mV*100), refusing\n", label, orig_raw * 625U + 50000U);
			return -EINVAL;
		}
	}

	pol = cpufreq_cpu_get(c->cpu_id);
	if (!pol) {
		CPU_RES("%s: cpufreq policy for cpu%u not found, refusing (cannot quiesce idx0)\n", label, c->cpu_id);
		return -ENODEV;
	}
	qret = quiesce_off_idx0(pol, cluster_id, &qreq);
	if (qret) {
		CPU_RES("%s: quiesce off idx0 failed (%d), nothing written\n", label, qret);
		cpufreq_cpu_put(pol);
		return qret;
	}

	writel(((u32)raw << 16) | mhz, w0p);
	prev_khz = c->opp_tbl[0].cpufreq_khz;
	c->opp_tbl[0].cpufreq_khz = mhz * 1000U;
	if (target_volt)
		c->opp_tbl[0].cpufreq_volt = raw * 625U + 50000U;
	sys_expose(pol, c, prev_khz, mhz * 1000U, label);
	freq_qos_remove_request(&qreq);
	cpufreq_cpu_put(pol);

	CPU_RES("%s: CSRAM idx0 now %u MHz / raw=%u (%u mV*100) (stock %u MHz / %u mV*100), policy/cpuinfo/EM updated. EEM may rewrite the volt field later\n",
		label, mhz, raw, raw * 625U + 50000U, orig_mhz, orig_raw * 625U + 50000U);
	return 0;
}

static __nocfi void restore_one_cluster(int cluster_id, const char *label)
{
	struct mt_cpu_dvfs_mirror *c;
	struct cpufreq_policy *pol = NULL;
	struct freq_qos_request qreq;
	bool quiesced = false;
	u32 *ref;
	u32 w0 = g_orig_w0[cluster_id];
	unsigned int prev_khz;

	if (cluster_id < 0 || cluster_id > 1 || !g_have_w0[cluster_id])
		return;
	ref = pvt_ref();
	c = cluster_get(cluster_id);
	if (!ref)
		return;

	if (c)
		pol = cpufreq_cpu_get(c->cpu_id);
	if (pol)
		quiesced = !quiesce_off_idx0(pol, cluster_id, &qreq);
	if (!quiesced)
		CPU_RES("%s: restoring WITHOUT quiesce (policy/idx unavailable)\n", label);

	writel(w0, (void __iomem *)(ref + cluster_id * 36));
	if (c && c->opp_tbl && c->nr_opp_tbl > 0 && c->nr_opp_tbl <= NR_FREQ_SANITY) {
		prev_khz = c->opp_tbl[0].cpufreq_khz;
		c->opp_tbl[0].cpufreq_khz  = (w0 & 0xFFFF) * 1000U;
		c->opp_tbl[0].cpufreq_volt = ((w0 >> 16) & 0xFFF) * 625U + 50000U;
		sys_expose(pol, c, prev_khz, (w0 & 0xFFFF) * 1000U, label);
	}
	if (quiesced)
		freq_qos_remove_request(&qreq);
	if (pol)
		cpufreq_cpu_put(pol);
	CPU_RES("%s: stock idx0 restored (%u MHz)\n", label, w0 & 0xFFFF);
}

/* Read-only dump of the CSRAM PVT table that the hybrid DVFS hardware really
 * uses. Layout taken from cpuhvfs_pvt_tbl_create() in mtk_cpufreq_hybrid.c:
 *   cluster c (0=LL, 1=L/big, 2=CCI) base = c*36 words
 *   word[base+i]    = (vproc_raw[11:0] << 16) | freq_MHz[15:0]
 *   word[base+16+i] = (clk_div << 16) | post_div
 * vproc mV*100 = raw*625 + 50000 (mt6358_transfer2volt in mt6768 platform.c).
 * NOTE: read only. Whether MCUPM re-reads this table on every request or only
 * at IPI_DVFS_INIT_PTBL is firmware behaviour and is NOT verifiable from source.
 */
static char cpu_pvt_table_buf[6144];

static __nocfi void build_cpu_pvt_table(void)
{
	size_t off = 0, sz = sizeof(cpu_pvt_table_buf);
	static const char *lbl[2] = { "little(LL)", "big(L)" };
	u32 *ref;
	int cid, i;

	if (!sym_recordRef_pp) {
		snprintf(cpu_pvt_table_buf, sz, "recordRef symbol not resolved (CONFIG_KALLSYMS_ALL / CPU_DVFS.ko?)\n");
		return;
	}
	ref = READ_ONCE(*sym_recordRef_pp);
	if (!ref || !is_vmalloc_addr(ref)) {
		snprintf(cpu_pvt_table_buf, sz, "recordRef=%p is not a valid ioremap address — refusing to read\n", ref);
		return;
	}

	for (cid = 0; cid < 2; cid++) {
		struct mt_cpu_dvfs_mirror *c = cluster_get(cid);
		int base = cid * 36;

		off += scnprintf(cpu_pvt_table_buf + off, sz - off, "%s CSRAM PVT:\n", lbl[cid]);
		for (i = 0; i < 16 && off < sz - 96; i++) {
			u32 w0 = readl((void __iomem *)(ref + base + i));
			u32 w1 = readl((void __iomem *)(ref + base + 16 + i));
			unsigned int mhz = w0 & 0xFFFF;
			unsigned int vraw = (w0 >> 16) & 0xFFF;
			bool have_ram = c && c->opp_tbl && i < c->nr_opp_tbl &&
					c->nr_opp_tbl <= NR_FREQ_SANITY;

			off += scnprintf(cpu_pvt_table_buf + off, sz - off,
				"  [%2d] %5u MHz  vraw=%3u (%6u mV*100)  posdiv=%u clkdiv=%u  ram=%s\n",
				i, mhz, vraw, vraw * 625 + 50000, w1 & 0xFF, (w1 >> 16) & 0xFF,
				!have_ram ? "n/a" :
				(c->opp_tbl[i].cpufreq_khz == mhz * 1000 ? "match" : "DIFF"));
		}
	}
}

static int cpu_pvt_table_get(char *buf, const struct kernel_param *kp)
{
	int n;

	mutex_lock(&oc_lock);
	build_cpu_pvt_table();
	n = scnprintf(buf, PAGE_SIZE, "%s", cpu_pvt_table_buf);
	mutex_unlock(&oc_lock);
	return n;
}
static const struct kernel_param_ops cpu_pvt_table_ops = { .get = cpu_pvt_table_get };
module_param_cb(cpu_pvt_table, &cpu_pvt_table_ops, NULL, 0444);
MODULE_PARM_DESC(cpu_pvt_table, "Read-only dump of the CSRAM PVT table used by hybrid DVFS (little+big), compared against cpu_dvfs[] RAM table");

/* Read-only: what the rest of the system actually sees for each cluster's top
 * frequency, next to what the hardware table (CSRAM) and the driver table say.
 * Counterpart of cpu_lut_table on the MT6789 module.
 */
static char cpu_sys_view_buf[2048];

static __nocfi void sys_view_one(int cid, const char *label, size_t *off)
{
	struct mt_cpu_dvfs_mirror *c = cluster_get(cid);
	struct cpufreq_policy *pol;
	struct cpufreq_stats_mirror *st;
	struct em_perf_domain *pd;
	struct cpufreq_frequency_table *dtbl;
	size_t sz = sizeof(cpu_sys_view_buf);
	unsigned int want, csram = 0, em = 0, stat0 = 0, ftab = 0, dt0 = 0, qos = 0;
	u32 *ref = pvt_ref();
	int bad = 0;

	if (!c || !c->opp_tbl || c->nr_opp_tbl <= 0 || c->nr_opp_tbl > NR_FREQ_SANITY) {
		*off += scnprintf(cpu_sys_view_buf + *off, sz - *off, "%s: cluster/opp_tbl unavailable\n", label);
		return;
	}
	pol = cpufreq_cpu_get(c->cpu_id);
	if (!pol) {
		*off += scnprintf(cpu_sys_view_buf + *off, sz - *off, "%s: no cpufreq policy for cpu%u\n", label, c->cpu_id);
		return;
	}

	want = c->opp_tbl[0].cpufreq_khz;
	dtbl = (struct cpufreq_frequency_table *)c->freq_tbl_for_cpufreq;
	dt0  = dtbl ? dtbl[0].frequency : 0;
	ftab = pol->freq_table ? pol->freq_table[0].frequency : 0;
	qos  = (pol->max_freq_req) ? (unsigned int)pol->max_freq_req->pnode.prio : 0;
	if (ref)
		csram = (readl((void __iomem *)(ref + cid * 36)) & 0xFFFF) * 1000U;
	pd = em_cpu_get(c->cpu_id);
	if (pd && pd->table && pd->nr_perf_states)
		em = pd->table[pd->nr_perf_states - 1].frequency;
	st = stats_get(pol);
	if (st)
		stat0 = st->freq_table[0];

	*off += scnprintf(cpu_sys_view_buf + *off, sz - *off,
		"%s (cpu%u), driver opp_tbl[0] = %u KHz\n"
		"  CSRAM idx0 (hw table)        %7u %s\n"
		"  driver freq_tbl_for_cpufreq  %7u %s\n"
		"  policy->freq_table[0]        %7u %s%s\n"
		"  cpuinfo_max_freq             %7u %s\n"
		"  scaling_max_freq (policy)    %7u %s\n"
		"  QoS max request              %7u %s\n"
		"  Energy Model top state       %7u %s\n"
		"  time_in_state top bucket     %7u %s\n"
		"  scaling_cur_freq (policy)    %7u\n",
		label, c->cpu_id, want,
		csram, csram == want ? "ok" : "DIFF",
		dt0, dt0 == want ? "ok" : "DIFF",
		ftab, ftab == want ? "ok" : "DIFF",
		(pol->freq_table == dtbl) ? "  (same array)" : "  (separate array)",
		pol->cpuinfo.max_freq, pol->cpuinfo.max_freq == want ? "ok" : "DIFF",
		pol->max, pol->max == want ? "ok" : "DIFF (cap: user/thermal?)",
		qos, qos == want ? "ok" : "DIFF (user set scaling_max?)",
		em, em == want ? "ok" : (em ? "DIFF" : "n/a"),
		stat0, st ? (stat0 == want ? "ok" : "DIFF") : "n/a (layout)",
		pol->cur);

	bad += (csram != want) + (dt0 != want) + (ftab != want) +
	       (pol->cpuinfo.max_freq != want) + (em != want) + (st && stat0 != want);
	*off += scnprintf(cpu_sys_view_buf + *off, sz - *off,
		"  => %s\n", bad ? "MISMATCH in hard views above" : "all hard views consistent");
	cpufreq_cpu_put(pol);
}

static int cpu_sys_view_get(char *buf, const struct kernel_param *kp)
{
	size_t off = 0;
	int n;

	mutex_lock(&oc_lock);
	cpu_sys_view_buf[0] = '\0';
	sys_view_one(MT_CPU_DVFS_LL, "little", &off);
	sys_view_one(MT_CPU_DVFS_L,  "big",    &off);
	n = scnprintf(buf, PAGE_SIZE, "%s", cpu_sys_view_buf);
	mutex_unlock(&oc_lock);
	return n;
}
static const struct kernel_param_ops cpu_sys_view_ops = { .get = cpu_sys_view_get };
module_param_cb(cpu_sys_view, &cpu_sys_view_ops, NULL, 0444);
MODULE_PARM_DESC(cpu_sys_view, "Read-only: per-cluster top frequency as seen by CSRAM, driver, policy, cpuinfo, QoS, Energy Model and time_in_state");

/*
 * QoS max guard.
 *
 * Something in userspace/vendor code rewrites the scaling_max_freq request
 * (policy->max_freq_req) back to the STOCK top frequency a few seconds after
 * cpu_oc_apply raised it (seen on device: 1900000 -> 1800000, 2100000 ->
 * 2000000 within 5 s). While that request says "stock", the governor can never
 * select the OC OPP even though every table is correct.
 *
 * Every cpu_guard_ms this puts the request back to the OC frequency, but ONLY
 * when it holds exactly the stock top value (= somebody reset it to default).
 * Any other value (user power-save cap, etc.) and every other QoS request
 * (thermal/PPM caps) is left alone: this never overrides a thermal cap, unlike
 * forcing policy->max directly.
 */
static unsigned int cpu_guard_ms = 200;
static unsigned int cpu_guard_hits;
static bool g_guard_stop;
static struct delayed_work g_cpu_guard_work;
module_param(cpu_guard_ms, uint, 0644);
MODULE_PARM_DESC(cpu_guard_ms, "Re-raise a stock-valued scaling_max QoS request to the OC top every N ms while CPU OC is active (0 = off; re-write cpu_oc_apply to re-arm)");
module_param(cpu_guard_hits, uint, 0444);
MODULE_PARM_DESC(cpu_guard_hits, "Read-only: how many times the guard had to lift a request that was reset to stock");

static __nocfi void cpu_guard_fn(struct work_struct *work)
{
	int cid, active = 0;

	if (g_guard_stop || !cpu_guard_ms)
		return;
	if (atomic_read(&oc_mt6768_suspended))
		goto rearm;
	if (!mutex_trylock(&oc_lock))	/* apply/restore in progress */
		goto rearm;

	for (cid = 0; cid < 2; cid++) {
		struct mt_cpu_dvfs_mirror *c;
		struct cpufreq_policy *pol;
		unsigned int oc, stock;

		if (!g_have_w0[cid])
			continue;
		c = cluster_get(cid);
		if (!c || !c->opp_tbl)
			continue;
		oc = c->opp_tbl[0].cpufreq_khz;
		stock = (g_orig_w0[cid] & 0xFFFF) * 1000U;
		if (oc <= stock)	/* not overclocked (restored) */
			continue;
		active++;

		pol = cpufreq_cpu_get(c->cpu_id);
		if (!pol)
			continue;
		if (pol->max_freq_req &&
		    pol->max_freq_req->pnode.prio == (int)stock) {
			freq_qos_update_request(pol->max_freq_req, oc);
			cpu_guard_hits++;
		}
		cpufreq_cpu_put(pol);
	}
	mutex_unlock(&oc_lock);
	if (!active)
		return;		/* nothing overclocked: stay idle until next apply */
rearm:
	if (!g_guard_stop && cpu_guard_ms && g_oc_wq)
		queue_delayed_work(g_oc_wq, &g_cpu_guard_work,
				   msecs_to_jiffies(cpu_guard_ms));
}

static int cpu_oc_apply_set(const char *val, const struct kernel_param *kp)
{
	int ret, rv;

	ret = kstrtoint(val, 0, &rv);
	if (ret)
		return ret;
	if (rv != 1)
		return 0;

	mutex_lock(&oc_lock);
	cpu_res_off = 0;
	cpu_oc_result[0] = '\0';
	if (cpu_ll_target_khz)
		apply_one_cluster(MT_CPU_DVFS_LL, "little", cpu_ll_target_khz, 0);
	else
		restore_one_cluster(MT_CPU_DVFS_LL, "little");
	if (cpu_b_target_khz)
		apply_one_cluster(MT_CPU_DVFS_L, "big", cpu_b_target_khz, 0);
	else
		restore_one_cluster(MT_CPU_DVFS_L, "big");
	mutex_unlock(&oc_lock);
	if (g_oc_wq && cpu_guard_ms && !g_guard_stop)
		mod_delayed_work(g_oc_wq, &g_cpu_guard_work, msecs_to_jiffies(cpu_guard_ms));
	return 0;
}
static int cpu_oc_apply_get(char *buf, const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE, "0\n");
}
static const struct kernel_param_ops cpu_oc_apply_ops = {
	.set = cpu_oc_apply_set,
	.get = cpu_oc_apply_get,
};
module_param_cb(cpu_oc_apply, &cpu_oc_apply_ops, &cpu_oc_apply, 0644);
MODULE_PARM_DESC(cpu_oc_apply, "Write 1 to apply CPU target frequencies to CSRAM PVT idx0 (both 0 = restore)");


static int __init oc_mt6768_init(void)
{
	unsigned long addr;
	int ret;

	pr_info("overclock_mt6768: init (GPU working_table patch + CPU CSRAM PVT idx0 patch)\n");

	ret = register_pm_notifier(&oc_mt6768_pm_nb);
	if (ret)
		pr_warn("oc_mt6768: register_pm_notifier failed (%d) — suspend guard inactive\n", ret);

	g_oc_wq = alloc_workqueue("oc_mt6768_wq", WQ_HIGHPRI | WQ_UNBOUND, 1);
	if (!g_oc_wq) {
		pr_err("oc_mt6768: failed to create workqueue\n");
		unregister_pm_notifier(&oc_mt6768_pm_nb);
		return -ENOMEM;
	}
	INIT_DELAYED_WORK(&g_gpu_pll_work, gpu_pll_work_fn);
	INIT_DELAYED_WORK(&g_cpu_guard_work, cpu_guard_fn);



	addr = resolve_ksym("gpufreq_get_working_table");
	if (addr)
		sym_gpufreq_get_working_table = (fn_get_wt_t)addr;
	else
		pr_warn("oc_mt6768: gpufreq_get_working_table not found — GPU module loaded?\n");

	addr = resolve_ksym("gpufreq_get_opp_num");
	if (addr)
		sym_gpufreq_get_opp_num = (fn_get_opp_num_t)addr;

	addr = resolve_ksym("gpufreq_get_signed_table");
	if (addr)
		sym_gpufreq_get_signed_table = (fn_get_wt_t)addr;
	else
		pr_warn("oc_mt6768: gpufreq_get_signed_table not found — signed table stays stock\n");

	addr = resolve_ksym("id_to_cpu_dvfs");
	if (addr) {
		sym_id_to_cpu_dvfs = (fn_id_to_cpu_dvfs_t)addr;
		pr_info("oc_mt6768: found id_to_cpu_dvfs — dump with cpu_opp_table / cpu_pvt_table\n");
	} else {
		pr_warn("oc_mt6768: id_to_cpu_dvfs not found — CPU_DVFS.ko not loaded? CPU side unavailable, GPU still works\n");
	}

	addr = resolve_ksym("cpuhvfs_get_cur_dvfs_freq_idx");
	if (addr)
		sym_cpuhvfs_get_cur_idx = (fn_cur_idx_t)addr;
	else
		pr_warn("oc_mt6768: cpuhvfs_get_cur_dvfs_freq_idx not found — CPU apply will refuse (cannot quiesce idx0)\n");

	addr = resolve_ksym("kallsyms_lookup_name");
	if (addr) {
		sym_kallsyms_lookup_name = (fn_kallsyms_lookup_name_t)addr;
		addr = sym_kallsyms_lookup_name("mtk_fh_set_rate");
		if (addr)

			sym_mtk_fh_set_rate_pp = (fn_mtk_fh_set_rate_t *)addr;
		else
			pr_warn("oc_mt6768: mtk_fh_set_rate not found — GPU PCW change fails safe\n");


		addr = sym_kallsyms_lookup_name("g_working_table");
		if (addr) {
			sym_ged_g_working_table = (struct gpufreq_opp_info **)addr;
			pr_info("oc_mt6768: found GED's g_working_table via kallsyms — current_freqency will reflect OC\n");
		} else {
			pr_warn("oc_mt6768: g_working_table not found via kallsyms (CONFIG_KALLSYMS_ALL off, ged.ko not loaded yet, or gpufreq v1 build) — GPU OC still applies at hardware level, but /sys/kernel/ged/hal/current_freqency will keep showing stock freq\n");
		}

		addr = sym_kallsyms_lookup_name("recordRef");
		if (addr)
			sym_recordRef_pp = (u32 **)addr;
		else
			pr_warn("oc_mt6768: recordRef not found — cpu_pvt_table unavailable\n");
	} else {
		pr_warn("oc_mt6768: kallsyms_lookup_name not found — GPU PCW change fails safe, and GED sync is unavailable\n");
	}

	memset(&krp_gpufreq_commit, 0, sizeof(krp_gpufreq_commit));
	krp_gpufreq_commit.kp.symbol_name = "gpufreq_commit";
	krp_gpufreq_commit.entry_handler  = krp_gpufreq_commit_entry;
	krp_gpufreq_commit.handler        = krp_gpufreq_commit_ret;
	krp_gpufreq_commit.data_size      = sizeof(struct gpu_commit_args);
	krp_gpufreq_commit.maxactive      = 4;
	ret = register_kretprobe(&krp_gpufreq_commit);
	if (ret < 0)
		pr_warn("oc_mt6768: kretprobe gpufreq_commit failed (%d)\n", ret);

	pr_info("oc_mt6768: CPU OC patches CSRAM PVT idx0, capped at stock idx0 +%d%% per cluster\n",
		MAX_OC_PERCENT_OVER_STOCK);

	pr_info("oc_mt6768: ready — sysfs at /sys/module/overclock_mt6768/parameters/\n");
	return 0;
}

static void __exit oc_mt6768_exit(void)
{
	pr_info("overclock_mt6768: unloading\n");

	unregister_pm_notifier(&oc_mt6768_pm_nb);

	mutex_lock(&oc_lock);
	g_gpu_oc_active = false;
	mutex_unlock(&oc_lock);

	g_guard_stop = true;
	if (g_oc_wq) {
		cancel_delayed_work_sync(&g_cpu_guard_work);
		cancel_delayed_work_sync(&g_gpu_pll_work);
		destroy_workqueue(g_oc_wq);
		g_oc_wq = NULL;
	}

	if (krp_gpufreq_commit.kp.addr)
		unregister_kretprobe(&krp_gpufreq_commit);

	mutex_lock(&oc_lock);
	gpu_restore_working_table();
	restore_one_cluster(MT_CPU_DVFS_LL, "little");
	restore_one_cluster(MT_CPU_DVFS_L,  "big");
	mutex_unlock(&oc_lock);

	if (g_apmixed_va) {
		iounmap(g_apmixed_va);
		g_apmixed_va = NULL;
	}

	pr_info("overclock_mt6768: unloaded, stock restored where patched\n");
}

module_init(oc_mt6768_init);
module_exit(oc_mt6768_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Anomali1304 (original MT6789/rock module); ported to MT6768/moon");
MODULE_DESCRIPTION("GPU (working_table) + CPU (cpu_dvfs[] top-index) OC for MediaTek MT6768 — Xiaomi 'moon', ported from the MT6789/rock module (CPU side rewritten for moon's actual mt-cpufreq driver). UNTESTED PORT for the CPU half — verify on real hardware before trusting it. GPU half has been dump-tested on real hardware.");
MODULE_VERSION("0.6.1-mt6768-driverpll-untested");