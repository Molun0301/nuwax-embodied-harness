//! Standalone production process for typed 3D person following.

use nuwax_nav_visual_servoing::lcm_owner::{PersonFollowLcmConfig, PersonFollowLcmOwner};
use nuwax_transport::LcmTransport;

const DEFAULT_LCM_URL: &str = "udpm://239.255.76.67:7667?ttl=1";

#[tokio::main]
async fn main() {
    if let Err(error) = run().await {
        eprintln!("nuwax-person-follow: {error}");
        std::process::exit(1);
    }
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let provider = std::env::var("NUWAX_LCM_URL").unwrap_or_else(|_| DEFAULT_LCM_URL.to_string());
    let transport = LcmTransport::new_real(&provider)?;
    let config = config_from_env();
    let owner = PersonFollowLcmOwner::new(transport, config)?;
    owner
        .run_until(async {
            let _ = tokio::signal::ctrl_c().await;
        })
        .await?;
    Ok(())
}

fn config_from_env() -> PersonFollowLcmConfig {
    let mut config = PersonFollowLcmConfig::default();
    override_string(
        "NUWAX_PERSON_FOLLOW_TARGET_CHANNEL",
        &mut config.target_channel,
    );
    override_string(
        "NUWAX_PERSON_FOLLOW_POINTCLOUD_CHANNEL",
        &mut config.pointcloud_channel,
    );
    override_string("NUWAX_PERSON_FOLLOW_TF_CHANNEL", &mut config.tf_channel);
    override_string(
        "NUWAX_PERSON_FOLLOW_COMMAND_CHANNEL",
        &mut config.command_channel,
    );
    config
}

fn override_string(name: &str, target: &mut String) {
    if let Ok(value) = std::env::var(name) {
        *target = value;
    }
}
