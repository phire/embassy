//! Boot Select button
//!
//! The RP2040 rom supports a BOOTSEL button that is used to enter the USB bootloader
//! if held during reset. To avoid wasting GPIO pins, the button is multiplexed onto
//! the CS pin of the QSPI flash, but that makes it quite expensive and complicated to
//! utilize outside of the rom's bootloader.
//!
//! This module provides functionality to poll BOOTSEL from an embassy application.

use rp_pac;
use rp_pac::io::vals::Oeover;

use crate::multicore;

/// Polls the BOOTSEL button. Returns true if the button is pressed.
///
/// Polling isn't cheap, this function waits for core 1 to finish it's current
/// task for any DMAs from flash to complete
///
/// Mostly safe. Must be called from core 0.
///
/// # Safety
///
/// Polling BOOTSEL requires the flash's QSPI bus to be idle. This function makes an
/// effort to ensure flash is idle, but the runtime checks can't be 100% sure.
///
/// There are a few edgecases the caller must avoid, such as:
///
///  * Core 1 might be running code other than the embassy executor, and which might access flash.
///  * DMA chaining might start new transfers that accesses flash
///  * Bypassing XIP and accessing SSI directly
///
pub unsafe fn poll_bootsel_unsafe() -> bool {
    assert!(rp_pac::SIO.cpuid().read() == 0, "Need to be on core 0");

    multicore::pause_core1();

    let button_state = critical_section::with(|cs| {
        // Wait for all DMA channels accessing flash to finish
        const SRAM_LOWER: u32 = 0x2000_0000;
        for n in 0..12 {
            let ch = rp_pac::DMA.ch(n);
            while ch.read_addr().read() < SRAM_LOWER && ch.ctrl_trig().read().busy() {}
        }
        // Wait for completion of any streaming reads
        while rp_pac::XIP_CTRL.stream_ctr().read().0 > 0 {}

        unsafe { poll_bootsel_ramfunc(&cs) }
    });

    multicore::resume_core1();

    button_state
}

/// This function runs from RAM so it can disable flash XIP
///
/// # Safety
///
/// The caller must ensure flash is idle and will remain idle.
///
/// Additionally, all functions called must be #[inline(always)], or in RAM.
///
/// The cortex-m-rt crate doesn't have a great way to mark a ram function yet,
/// so we put it in the .data section and make sure it isn't inlined
#[inline(never)]
#[link_section = ".data.ram_func"]
unsafe fn poll_bootsel_ramfunc(_: &critical_section::CriticalSection<'_>) -> bool {
    // Make sure the XIP controller is idle
    loop {
        let xip_status = rp_pac::XIP_CTRL.stat().read();
        if xip_status.fifo_empty() && xip_status.flush_ready() { break; }
    }

    let chip_select = rp_pac::IO_QSPI.gpio(1);

    // The BOOTSEL pulls the flash's CS line low though a 1K resistor.
    // this is weak enough to avoid disrupting normal operation.
    // But, if we disable CS's output drive allow it to float...
    chip_select.ctrl().write(|g| g.set_oeover(Oeover::DISABLE));

    // ...then wait for the state to settle...
    // (sleep might be in flash, use a delay loop)
    cortex_m::asm::delay(2000);

    // ...we can read the current state of the button (active low)
    let button_state = !chip_select.status().read().infrompad();

    // Finally, restore CS to normal operation so XIP can continue
    chip_select.ctrl().write(|g| g.set_oeover(Oeover::NORMAL));

    button_state
}
