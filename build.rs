use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn Error>> {
    embuild::espidf::sysenv::output();

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR")?);
    let config_path = manifest_dir.join("app_config.toml");
    println!("cargo:rerun-if-changed={}", config_path.display());

    let config_contents = fs::read_to_string(&config_path)?;
    let table: toml::value::Table = toml::from_str(&config_contents)?;

    let get = |key: &str| {
        table
            .get(key)
            .and_then(|value| value.as_str())
            .unwrap_or_else(|| panic!("app_config.toml must define `{}` as a string", key))
    };

    let generated = format!(
        "pub const BOT_TOKEN: &str = {bot_token:?};\n\
         pub const GIST_URL: &str = {gist_url:?};\n\
         pub const WIFI_SSID: &str = {wifi_ssid:?};\n\
         pub const WIFI_PASS: &str = {wifi_pass:?};\n",
        bot_token = get("bot_token"),
        gist_url = get("gist_url"),
        wifi_ssid = get("wifi_ssid"),
        wifi_pass = get("wifi_pass"),
    );

    let out_dir = PathBuf::from(env::var("OUT_DIR")?);
    fs::write(out_dir.join("app_config.rs"), generated)?;

    Ok(())
}
