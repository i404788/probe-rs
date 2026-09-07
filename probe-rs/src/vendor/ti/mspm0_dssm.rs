//! Texas Instruments MSPM0 Debug SubSystem Mailbox (DSSM) recovery.
//!
//! A locked MSPM0 device (for example after a failed write-after-erase, or when
//! a debug lock is programmed into NONMAIN) exposes no MEM-AP: the core cannot
//! be examined and the flash cannot be driven through the usual flash
//! algorithms. The only remaining debug access is the SECAP access port
//! (APSEL 2), through which the ROM bootloader accepts DSSM mailbox commands.
//!
//! A DSSM command is executed by the ROM on the next reset: the debugger
//! writes the command into the SECAP mailbox, toggles nRST, and the ROM picks
//! the command up while booting, reporting the outcome back through the
//! mailbox. The factory reset command erases the main and non-main flash and
//! restores the device to its out-of-the-box (unlocked) state.
//!
//! The command values and the register layout are transcribed from TI's
//! ti-openocd port (`tcl/target/ti_mspm0.cfg`), which follows the MSPM0
//! Technical Reference Manual "Debug SubSystem Mailbox" chapter.
//!
//! # Limitations
//!
//! Password-protected DSSM commands are not supported: only the commands that
//! the ROM accepts without authentication are issued.

use std::time::{Duration, Instant};

use super::sequences::mspm0::MSPM0;
use crate::architecture::arm::communication_interface::ArmDebugInterface;
use crate::architecture::arm::dp::DpAddress;
use crate::architecture::arm::{ArmError, FullyQualifiedApAddress};
use crate::probe::Probe;

/// The SECAP access port: APSEL 2 on every MSPM0 device.
fn secap() -> FullyQualifiedApAddress {
    FullyQualifiedApAddress::v1_with_default_dp(2)
}

/// The CFG-AP: APSEL 1 on every MSPM0 device. It reports the boot diagnostic.
fn cfg_ap() -> FullyQualifiedApAddress {
    FullyQualifiedApAddress::v1_with_default_dp(1)
}

/// SECAP `TDR` — transfer data register (command payload).
const SECAP_TDR: u64 = 0x00;
/// SECAP `TCR` — transfer command register (the DSSM command).
const SECAP_TCR: u64 = 0x04;
/// SECAP `RXD` — receive data register (response payload).
const SECAP_RXD: u64 = 0x08;
/// SECAP `RCR` — receive control register (status flags).
const SECAP_RCR: u64 = 0x0C;

/// SECAP `RCR` bit 0: `RX_VALID`, set by the ROM once a response is pending.
///
/// Reading `SECAP::RXD` clears this bit.
const RCR_RX_VALID: u32 = 1 << 0;

/// DSSM command `0x0108`: ask the ROM to start the bootloader after reset.
pub const DSSM_CMD_START_BOOTLOADER: u32 = 0x0108;
/// DSSM command `0x020a`: factory reset.
///
/// Erases the main and non-main flash and clears all debug security settings,
/// recovering devices that are otherwise bricked. Requires a subsequent reset
/// (performed as part of the command flow) to take effect.
pub const DSSM_CMD_FACTORY_RESET: u32 = 0x020a;
/// DSSM command `0x020c`: mass erase of the main flash.
pub const DSSM_CMD_MASS_ERASE: u32 = 0x020c;

/// The value the ROM writes to `SECAP::RXD` after successfully executing a
/// command.
const DSSM_RESPONSE_SUCCESS: u32 = 0x1_0003;

/// Time the ROM needs between the mailbox write and sampling it after reset.
const MAILBOX_SETTLE: Duration = Duration::from_millis(1000);
/// How long the ROM may take to acknowledge a command after nRST.
const RESPONSE_TIMEOUT: Duration = Duration::from_millis(2000);
/// Interval between `RCR` polls while waiting for the ROM response.
const RESPONSE_POLL_INTERVAL: Duration = Duration::from_millis(10);
/// How long nRST is held asserted.
const NRST_ASSERT: Duration = Duration::from_millis(100);
/// Time for the device to boot after nRST is released.
const RESET_SETTLE: Duration = Duration::from_millis(100);

/// `DAP_SWJ_Pins` pin select/output bit for nRESET.
const SWJ_PIN_NRESET: u32 = 1 << 7;

/// CFG-AP register `0x10`: the boot diagnostic, written by the bootcode.
const BOOTDIAG: u64 = 0x10;
/// Boot diagnostic value TI's packs associate with a corrupted NONMAIN.
const BOOTDIAG_NONMAIN_CORRUPTED: u32 = 0x36;

/// Whether the given chip name belongs to the TI MSPM0/MSPS family.
///
/// Uses the same name prefixes as [`super::TexasInstruments::try_create_debug_sequence`].
pub fn is_mspm0_family(chip_name: &str) -> bool {
    chip_name.starts_with("MSPM0") || chip_name.starts_with("MSPS")
}

/// Execute a single DSSM mailbox command through the SECAP access port.
///
/// The flow is the one TI's tools use:
///
/// 1. Write the command to `SECAP::TCR` and `0` to `SECAP::TDR`, then clear any
///    stale response by reading `RXD`/`RCR`.
/// 2. Toggle nRST. The ROM samples the mailbox when it boots after the reset.
/// 3. Poll `SECAP::RCR` for `RX_VALID`, then read `SECAP::RXD` to fetch (and
///    acknowledge) the response.
/// 4. Toggle nRST once more so the device leaves the ROM's command handling
///    with a clean slate.
///
/// While the bootcode executes the command, the PWR-AP handling in the debug
/// sequence must not run: its system reset would abort a mass erase or factory
/// reset mid-operation. `sequence` is therefore suppressed for the duration of
/// the poll phase.
pub fn dssm_command(
    interface: &mut dyn ArmDebugInterface,
    sequence: &super::sequences::mspm0::MSPM0,
    command: u32,
) -> Result<(), ArmError> {
    let secap = secap();

    tracing::debug!("MSPM0 DSSM: issuing command {:#06x}", command);

    interface.write_raw_ap_register(&secap, SECAP_TCR, command)?;
    interface.write_raw_ap_register(&secap, SECAP_TDR, 0)?;

    // Clear any response left over from a previous command.
    let _ = interface.read_raw_ap_register(&secap, SECAP_RXD);
    let _ = interface.read_raw_ap_register(&secap, SECAP_RCR);

    // Give the ROM time to latch the command, then reset into it.
    std::thread::sleep(MAILBOX_SETTLE);
    toggle_reset(interface)?;

    // The reset drops the SWD connection: bring the debug port back up so the
    // mailbox can be polled. The PWR-AP recovery in the debug sequence must not
    // run while the bootcode is working, so suppress it until we are done.
    sequence.suppress_recovery();
    let response = poll_response(interface, command);
    sequence.resume_recovery();
    response?;

    // Make sure the ROM is done, then reset back to a sane system state.
    std::thread::sleep(MAILBOX_SETTLE);
    toggle_reset(interface)?;
    std::thread::sleep(MAILBOX_SETTLE);

    Ok(())
}

/// Poll for the response to a DSSM command, re-establishing the SWD link as
/// needed, and validate the command echo.
fn poll_response(interface: &mut dyn ArmDebugInterface, command: u32) -> Result<(), ArmError> {
    // The reset drops the SWD connection: bring the debug port back up so the
    // mailbox can be polled. If the link is not back yet, the poll loop below
    // keeps retrying.
    if let Err(error) = interface.reinitialize() {
        tracing::debug!("MSPM0 DSSM: reconnect after reset failed: {error}");
    }

    let deadline = Instant::now() + RESPONSE_TIMEOUT;
    let rcr = loop {
        match interface.read_raw_ap_register(&secap(), SECAP_RCR) {
            Ok(rcr) if rcr & RCR_RX_VALID != 0 => break rcr,
            Ok(_) => {}
            Err(error) => {
                // The link may still be coming back up after the reset.
                tracing::debug!("MSPM0 DSSM: SECAP read failed, reconnecting: {error}");
                if let Err(error) = interface.reinitialize() {
                    tracing::debug!("MSPM0 DSSM: reconnect failed: {error}");
                }
            }
        }
        if Instant::now() >= deadline {
            log_timeout_diagnostics(interface);
            return Err(ArmError::Timeout);
        }
        std::thread::sleep(RESPONSE_POLL_INTERVAL);
    };

    let rxd = interface.read_raw_ap_register(&secap(), SECAP_RXD)?;

    // The command echo shares the low bits of `RCR` with the `RX_VALID` flag,
    // which is still set at this point (reading `RXD` above cleared it on the
    // device, but our `RCR` sample predates that read).
    let rcr_echo = rcr & 0xff & !RCR_RX_VALID;
    if rxd != DSSM_RESPONSE_SUCCESS {
        tracing::error!("MSPM0 DSSM: command {command:#06x} failed, RXD: {rxd:#010x}");
        return Err(ArmError::Other(format!(
            "MSPM0 DSSM command {command:#06x} failed: RXD: {rxd:#010x}, RCR: {rcr:#010x}"
        )));
    }
    if rcr_echo != (command & 0xff) {
        tracing::error!(
            "MSPM0 DSSM: command echo mismatch, expected {:#x}, got {:#x}",
            command & 0xff,
            rcr_echo
        );
        return Err(ArmError::Other(format!(
            "MSPM0 DSSM command {command:#06x} failed: unexpected RCR echo {rcr:#010x}"
        )));
    }

    tracing::info!("MSPM0 DSSM: command {command:#06x} acknowledged by the ROM");

    Ok(())
}

/// Log diagnostics when the bootcode does not respond to a DSSM command.
///
/// A silent bootcode usually means one of:
///
/// - the probe cannot actually toggle the nRST line (the command is never
///   handed to the bootcode, which only samples the mailbox during boot), or
/// - the device's NONMAIN configuration rejects mailbox commands outright
///   (e.g. "Debug Disabled", or a corrupted NONMAIN whose pattern-match fields
///   fail safe to a maximally secure configuration).
///
/// The CFG-AP boot diagnostic is read for the latter case, where TI's packs
/// report e.g. `0x36` for a corrupted NONMAIN.
fn log_timeout_diagnostics(interface: &mut dyn ArmDebugInterface) {
    let bootdiag = interface.read_raw_ap_register(&cfg_ap(), BOOTDIAG);
    match bootdiag {
        Ok(value) => {
            if value == BOOTDIAG_NONMAIN_CORRUPTED {
                tracing::warn!(
                    "MSPM0 DSSM: no response from the bootcode; CFG-AP boot diagnostic is \
                     {value:#010x}: the NONMAIN configuration appears to be corrupted. If the \
                     device still does not respond, it may not be recoverable over SWD."
                );
            } else {
                tracing::warn!(
                    "MSPM0 DSSM: no response from the bootcode; CFG-AP boot diagnostic is \
                     {value:#010x}. Make sure the probe's nRST line is wired to the target and \
                     can actually be driven low: the command is only serviced by the bootcode \
                     during boot."
                );
            }
        }
        Err(error) => tracing::debug!("MSPM0 DSSM: failed to read the boot diagnostic: {error}"),
    }
}

/// Assert nRST for a short time, then release it.
///
/// The device boots the ROM (which processes any pending DSSM command) as nRST
/// is released.
fn toggle_reset(interface: &mut dyn ArmDebugInterface) -> Result<(), ArmError> {
    interface
        .swj_pins(0, SWJ_PIN_NRESET, 0)
        .map_err(ArmError::from)?;
    std::thread::sleep(NRST_ASSERT);
    interface
        .swj_pins(SWJ_PIN_NRESET, SWJ_PIN_NRESET, 0)
        .map_err(ArmError::from)?;
    std::thread::sleep(RESET_SETTLE);

    Ok(())
}

/// Run a DSSM factory reset against the device attached to `probe`.
///
/// The probe is attached without a target: a locked MSPM0 has no MEM-AP, so no
/// [`crate::Session`] can be created. The returned probe is detached and can be
/// reused for a regular attach (which now succeeds because the device is
/// unlocked and blank).
///
/// # Errors
///
/// Returns an error if the probe does not support ARM debugging or nRST
/// control, if the device does not respond, or if the ROM rejects the command.
/// Note that the factory reset erases the entire main *and* non-main flash.
pub fn factory_reset(mut probe: Probe) -> Result<Probe, crate::Error> {
    // MSPM0 only supports SWD, and the probe protocol is not selected by the
    // unspecified attach below.
    probe.select_protocol(crate::probe::WireProtocol::Swd)?;
    probe.attach_to_unspecified()?;

    // The MSPM0 sequence keeps the device out of DEEPSLEEP while we talk to it;
    // a blank device parks itself in STANDBY after a few seconds otherwise.
    let sequence = MSPM0::create("MSPM0".to_string());
    let mut interface = match probe.try_into_arm_debug_interface(sequence.clone()) {
        Ok(interface) => interface,
        Err((probe, error)) => {
            let mut probe = probe;
            let _ = probe.detach();
            return Err(error.into());
        }
    };

    let result = dssm_command(&mut *interface, &sequence, DSSM_CMD_FACTORY_RESET);

    // First AP access triggered the DP setup implicitly; make sure the debug
    // port is selected even if the command failed before that point, so the
    // interface can be closed cleanly.
    if interface.current_debug_port().is_none() {
        let _ = interface.select_debug_port(DpAddress::Default);
    }

    let mut probe = interface.close();
    let _ = probe.detach();

    result?;
    Ok(probe)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The command values TI's ti-openocd port (`tcl/target/ti_mspm0.cfg`)
    /// sends, pinned so a transcription error cannot silently change what goes
    /// on the wire.
    #[test]
    fn dssm_commands_match_ti_openocd() {
        assert_eq!(DSSM_CMD_START_BOOTLOADER, 0x0108);
        assert_eq!(DSSM_CMD_FACTORY_RESET, 0x020a);
        assert_eq!(DSSM_CMD_MASS_ERASE, 0x020c);
        assert_eq!(DSSM_RESPONSE_SUCCESS, 0x10003);
        assert_eq!(RCR_RX_VALID, 0x1);
        assert_eq!(SWJ_PIN_NRESET, 0x80);
    }

    /// SECAP register offsets from the MSPM0 Technical Reference Manual /
    /// ti-openocd's `ti_mspm0.cfg`.
    #[test]
    fn secap_register_offsets() {
        use crate::architecture::arm::ApAddress;
        assert_eq!(SECAP_TDR, 0x0);
        assert_eq!(SECAP_TCR, 0x4);
        assert_eq!(SECAP_RXD, 0x8);
        assert_eq!(SECAP_RCR, 0xc);
        assert_eq!(secap().ap(), &ApAddress::V1(2));
    }

    /// Guards the chip-name prefix matching used by the CLI: every built-in
    /// MSPM0/MSPS target must be detected, and other families must not be.
    #[test]
    fn family_detection_matches_vendor_prefixes() {
        for name in [
            "MSPM0L1306",
            "MSPM0C1104",
            "MSPM0G3507",
            "MSPM0G3519",
            "MSPS003F4",
        ] {
            assert!(
                is_mspm0_family(name),
                "{name} should be detected as MSPM0/MSPS family"
            );
        }

        for name in ["nRF52840", "STM32F103C8", "RP2040", "CC1352R1F3"] {
            assert!(
                !is_mspm0_family(name),
                "{name} should not be detected as MSPM0/MSPS family"
            );
        }
    }
}
