use std::path::PathBuf;

use probe_rs_rpc_client::RpcClient;

use crate::util::cli;
use crate::util::common_options::BinaryDownloadOptions;
use crate::util::common_options::ProbeOptions;
use probe_rs_rpc::format::FormatOptions;

#[derive(clap::Parser)]
pub struct Cmd {
    #[clap(flatten)]
    pub probe_options: ProbeOptions,

    /// The path to the file to be downloaded to the flash
    pub path: PathBuf,

    #[clap(flatten)]
    pub download_options: BinaryDownloadOptions,

    #[clap(flatten)]
    pub format_options: FormatOptions,

    /// Start the firmware after the download.
    ///
    /// The target gets a reset only if the firmware needs one to start.
    #[clap(long, help_heading = "DOWNLOAD CONFIGURATION")]
    pub start: bool,
}

impl Cmd {
    pub async fn run(self, client: RpcClient) -> anyhow::Result<()> {
        let Self {
            probe_options,
            path,
            mut download_options,
            format_options,
            start,
        } = self;

        // On TI MSPM0 devices, a chip erase is a DSSM factory reset: it erases
        // the main and non-main flash and clears the debug security settings,
        // which also recovers locked devices. The flash loader does not need
        // to erase anything afterwards.
        let chip_is_mspm0 = probe_options
            .chip
            .as_deref()
            .is_some_and(probe_rs::vendor::ti::mspm0_dssm::is_mspm0_family);
        let factory_reset = download_options.chip_erase && chip_is_mspm0;
        if factory_reset {
            download_options.chip_erase = false;
        }

        let session = cli::attach_probe(&client, probe_options, None, false, factory_reset).await?;

        let boot_info = cli::flash(
            &session,
            &path,
            format_options,
            download_options,
            None,
            None,
        )
        .await?;

        if start {
            session.boot(boot_info, 0).await?;
        }

        Ok(())
    }
}
