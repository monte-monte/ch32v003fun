all : flash

TARGET:=spi_dma_chain
TARGET_MCU:=CH32V203
# TARGET_MCU:=CH32V307
# TARGET_MCU:=CH32V103
ENABLE_FPU:=0
include ../../ch32fun/ch32fun.mk
# CFLAGS+=-g -flto -ffunction-sections -fdata-sections -fmessage-length=0 -msmall-data-limit=8 -faggressive-loop-optimizations -fallocation-dce -fauto-inc-dec -fbit-tests -fbranch-count-reg -fcaller-saves -fcode-hoisting -fcombine-stack-adjustments -fcompare-elim -fcprop-registers -fcrossjumping -fcse-follow-jumps -fdce -fdefer-pop -fdelete-null-pointer-checks -fdevirtualize -fdevirtualize-speculatively -fdse -fearly-inlining -fexceptions -fexpensive-optimizations -ffast-math -fforward-propagate -ffp-contract=fast -ffp-int-builtin-inexact -ffunction-cse -fgcse -fgcse-lm -fguess-branch-probability -fhandle-exceptions -fhoist-adjacent-loads -fif-conversion -fif-conversion2 -findirect-inlining -finline -finline-atomics -finline-functions -finline-functions-called-once -finline-small-functions -fipa-bit-cp -fipa-cp -fipa-icf -fipa-icf-functions -fipa-icf-variables -fipa-modref -fipa-profile -fipa-pure-const -fipa-ra -fipa-reference -fipa-reference-addressable -fipa-sra -fipa-stack-alignment -fipa-strict-aliasing -fipa-vrp -fira-algorithm=CB -fira-hoist-pressure -fira-region=one -fira-share-save-slots -fira-share-spill-slots -fisolate-erroneous-paths-dereference -fivopts -fjump-tables -flifetime-dse=2 -flra-remat -fmath-errno -fmove-loop-invariants -fmove-loop-stores -fomit-frame-pointer -fopenmp-target-simd-clone=none -foptimize-sibling-calls -fpartial-inlining -fpeephole -fpeephole2 -fplt -fprintf-return-value -free -freg-struct-return -freorder-blocks -freorder-blocks-algorithm=simple -freorder-functions -frerun-cse-after-loop -fsched-critical-path-heuristic -fsched-dep-count-heuristic -fsched-group-heuristic -fsched-interblock -fsched-last-insn-heuristic -fsched-rank-heuristic -fsched-spec -fsched-spec-insn-heuristic -fsched-stalled-insns-dep -fschedule-fusion -fschedule-insns2 -fsection-anchors -fsemantic-interposition -fshort-enums -fshrink-wrap -fshrink-wrap-separate -fsigned-zeros -fsimd-cost-model=unlimited -fsplit-ivs-in-unroller -fsplit-wide-types -fssa-backprop -fssa-phiopt -fstack-check=generic -fstack-reuse=all -fstdarg-opt -fstore-merging -fstrict-aliasing -fstrict-volatile-bitfields -fthread-jumps -ftoplevel-reorder -ftrapping-math -ftree-bit-ccp -ftree-builtin-call-dce -ftree-ccp -ftree-ch -ftree-coalesce-vars -ftree-copy-prop -ftree-dce -ftree-dominator-opts -ftree-dse -ftree-forwprop -ftree-fre -ftree-loop-distribute-patterns -ftree-loop-if-convert -ftree-loop-im -ftree-loop-ivcanon -ftree-loop-optimize -ftree-parallelize-loops=1 -ftree-phiprop -ftree-pre -ftree-pta -ftree-reassoc -ftree-scev-cprop -ftree-sink -ftree-slsr -ftree-sra -ftree-switch-conversion -ftree-tail-merge -ftree-ter -ftree-vrp -ftrivial-auto-var-init=uninitialized -fvect-cost-model=very-cheap
# CFLAGS+=-Q -v -g -flto -ffunction-sections -fdata-sections -fmessage-length=0 -msmall-data-limit=8 -O2
# CFLAGS+=-O2 -fno-tree-slp-vectorize -fno-tree-loop-vectorize -fno-schedule-insns -freorder-blocks-algorithm=simple -fno-optimize-strlen -fopenmp-target-simd-clone=none -fno-align-loops -fno-align-labels -fno-align-jumps -fno-align-functions
# CFLAGS+=-Os -fno-ipa-icf-variables -fno-ipa-icf-functions
CFLAGS+=-Wno-switch -Wno-attributes -fno-ipa-icf-variables -fno-ipa-icf-functions
# CFLAGS+=-O0

flash : cv_flash
clean : cv_clean
macro : $(FILES_TO_COMPILE) $(LINKER_SCRIPT) $(EXTRA_ELF_DEPENDENCIES)
	$(PREFIX)-gcc -E spi_dma_chain.c -o spi_dma_chain.i $(CFLAGS) $(LDFLAGS)