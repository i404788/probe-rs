use crate::util::{cli, common_options::ProbeOptions, flash::CliProgressBars};
use probe_rs_rpc_client::RpcClient;

#[derive(clap::Parser)]
pub struct Cmd {
    #[clap(flatten)]
    common: ProbeOptions,

    #[arg(long, help_heading = "DOWNLOAD CONFIGURATION")]
    pub disable_progressbars: bool,

    /// Whether to read the RTT output from the flash loader, if available.
    #[clap(long)]
    pub read_flasher_rtt: bool,

    /// On TI MSPM0 devices, erase via a DSSM factory reset instead.
    ///
    /// This erases the main and non-main flash and clears the debug security
    /// settings, which also recovers locked devices. Requires the probe's
    /// nRESET pin to be wired to the target.
    #[arg(long, help_heading = "DOWNLOAD CONFIGURATION")]
    pub chip_erase: bool,
}

impl Cmd {
    pub async fn run(self, client: RpcClient) -> anyhow::Result<()> {
        let chip_is_mspm0 = self
            .common
            .chip
            .as_deref()
            .is_some_and(probe_rs::vendor::ti::mspm0_dssm::is_mspm0_family);
        let factory_reset = self.chip_erase && chip_is_mspm0;

        let session = cli::attach_probe(&client, self.common, None, false, factory_reset).await?;

        if factory_reset {
            // The DSSM factory reset already erased everything.
            return Ok(());
        }

        let pb = if self.disable_progressbars {
            None
        } else {
            Some(CliProgressBars::new())
        };

        session
            .erase_all(self.read_flasher_rtt, async move |event| {
                if let Some(pb) = pb.as_ref() {
                    pb.handle(event);
                }
            })
            .await?;

        Ok(())
    }
}
