use std::collections::{HashMap, VecDeque};
use std::thread;
use std::time::{Duration, Instant};
use std::vec::Vec;

use anyhow::{anyhow, Context, Result};
use core::convert::{Infallible, TryInto};
use core::ptr;
use core::sync::atomic::{AtomicBool, Ordering};
use embedded_svc::http::client::Client as HttpClient;
use embedded_svc::http::Method;
use esp_idf_hal::delay::TickType;
use esp_idf_hal::gpio::{AnyIOPin, Gpio0, Gpio19, Input, Output, PinDriver, Pull};
use esp_idf_hal::i2c::{self, APBTickType, I2cDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::uart::{self, UartDriver};
use esp_idf_hal::units::Hertz;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::http::client::{
    Configuration as HttpConfiguration, EspHttpConnection, FollowRedirectsPolicy,
};
use esp_idf_svc::log::EspLogger;
use esp_idf_svc::nvs::{EspDefaultNvs, EspDefaultNvsPartition, EspNvs};
use esp_idf_svc::wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use log::{error, info, warn};
use pn532::i2c::I2CInterface;
use pn532::nb;
use pn532::requests::SAMMode;
use pn532::{CountDown, Error as PnError, IntoDuration, Pn532, Request};
use portable_atomic::AtomicU64;
use serde_json;
use std::sync::{Mutex, OnceLock};

use esp_idf_svc::sys;

mod build_config {
    include!(concat!(env!("OUT_DIR"), "/app_config.rs"));
}

use build_config::{BOT_TOKEN, GIST_URL, WIFI_PASS, WIFI_SSID};

const TELEGRAM_PRIVATE_CHAT_ID: &str = "-1001521238614";
const TELEGRAM_PUBLIC_CHAT_ID: &str = "-1001742786420";
const PN532_BUFFER: usize = 128;
const LISTEN_TIMEOUT_MS: u64 = 100;
const WATCHDOG_TIMEOUT_SECS: u64 = 60;
const UID_REPEAT_SUPPRESSION_SECS: u64 = 10;
const PN532_RETRY_DELAY_MS: u64 = 500;
const NVS_NAMESPACE: &str = "akiba";
const NVS_KEYS_KEY: &str = "uids";
const NVS_MAX_CACHE_SIZE: usize = 8192;
const WATCHDOG_MONITOR_MARGIN_SECS: u64 = 5;
const WATCHDOG_MONITOR_POLL_MS: u64 = 500;
const MANUAL_REBOOT_HOLD_SECS: u64 = 2;
const LOG_PERIODIC_FLUSH_SECS: u64 = 600;
const HTTP_RESPONSE_LOG_LIMIT: usize = 8192;

const HTTP_CONFIG: HttpConfiguration = HttpConfiguration {
    timeout: Some(Duration::from_secs(3)),
    crt_bundle_attach: Some(sys::esp_crt_bundle_attach),
    buffer_size: Some(16 * 1024),
    buffer_size_tx: Some(16 * 1024),
    follow_redirects_policy: FollowRedirectsPolicy::FollowGetHead,
    client_certificate: None,
    private_key: None,
    use_global_ca_store: false,
    raw_request_body: false,
};

const LOG_NAMESPACE: &str = "logbuf";
const LOG_STORAGE_KEY: &str = "buffer";
const LOG_BUFFER_CAPACITY: usize = 64;
const LOG_ENTRY_MAX_LEN: usize = 192;
const LOG_STORAGE_LIMIT: usize = 3500;

static TELEMETRY_LOGGER: OnceLock<TelemetryLogger> = OnceLock::new();
static LAST_WDT_FEED_US: AtomicU64 = AtomicU64::new(0);
static WATCHDOG_MONITOR_TRIGGERED: AtomicBool = AtomicBool::new(false);

fn telemetry_logger() -> &'static TelemetryLogger {
    TELEMETRY_LOGGER.get_or_init(TelemetryLogger::new)
}

unsafe extern "C" fn flush_logs_on_shutdown() {
    if let Some(logger) = TELEMETRY_LOGGER.get() {
        logger.flush_on_shutdown();
    }
}

struct TelemetryLogger {
    inner: EspLogger,
    buffer: Mutex<LogBuffer>,
    storage: Mutex<Option<EspDefaultNvs>>,
    panic_hook_installed: AtomicBool,
}

struct LogBuffer {
    entries: VecDeque<String>,
    dirty: bool,
}

impl LogBuffer {
    fn new() -> Self {
        Self {
            entries: VecDeque::with_capacity(LOG_BUFFER_CAPACITY),
            dirty: false,
        }
    }
}

impl TelemetryLogger {
    fn new() -> Self {
        Self {
            inner: EspLogger::new(),
            buffer: Mutex::new(LogBuffer::new()),
            storage: Mutex::new(None),
            panic_hook_installed: AtomicBool::new(false),
        }
    }

    fn install(&'static self, partition: EspDefaultNvsPartition) -> Result<()> {
        self.ensure_storage(partition)?;
        log::set_logger(self).map_err(|_| anyhow!("logger already initialized"))?;
        log::set_max_level(self.inner.get_max_level());
        self.install_panic_hook();
        unsafe {
            sys::esp!(sys::esp_register_shutdown_handler(Some(
                flush_logs_on_shutdown
            )))?;
        }
        Ok(())
    }

    fn ensure_storage(&self, partition: EspDefaultNvsPartition) -> Result<()> {
        let mut guard = self.storage.lock().unwrap();
        if guard.is_none() {
            let nvs = EspNvs::new(partition, LOG_NAMESPACE, true)?;
            *guard = Some(nvs);
        }
        Ok(())
    }

    fn capture(&self, record: &log::Record) {
        let timestamp = unsafe { sys::esp_log_timestamp() };
        let mut entry = format!(
            "{:>10} {} {}: {}",
            timestamp,
            record.level(),
            record.target(),
            record.args()
        );
        if entry.len() > LOG_ENTRY_MAX_LEN {
            entry.truncate(LOG_ENTRY_MAX_LEN);
        }
        let mut buffer = self.buffer.lock().unwrap();
        if buffer.entries.len() == LOG_BUFFER_CAPACITY {
            buffer.entries.pop_front();
        }
        buffer.entries.push_back(entry);
        buffer.dirty = true;
    }

    fn recent_logs(buffer: &LogBuffer) -> String {
        let mut text = String::new();
        for entry in buffer.entries.iter().rev() {
            if !text.is_empty() {
                text.push('\n');
            }
            text.push_str(entry);
        }
        if text.len() > LOG_STORAGE_LIMIT {
            text.truncate(LOG_STORAGE_LIMIT);
        }
        text
    }

    fn persist_to_nvs(&self) -> Result<()> {
        info!("persist_to_nvs: enter");
        let data = {
            let mut buffer = self.buffer.lock().unwrap();
            if !buffer.dirty {
                return Ok(());
            }
            buffer.dirty = false;
            Self::recent_logs(&buffer).into_bytes()
        };

        let mut storage = self.storage.lock().unwrap();
        if let Some(nvs) = storage.as_mut() {
            if data.is_empty() {
                let _ = nvs.remove(LOG_STORAGE_KEY);
            } else {
                nvs.set_raw(LOG_STORAGE_KEY, &data)
                    .map_err(anyhow::Error::from)?;
            }
        }
        info!("persist_to_nvs: saved {} bytes", data.len());

        Ok(())
    }

    fn take_persisted_logs(&self) -> Result<Option<String>> {
        let mut storage = self.storage.lock().unwrap();
        let Some(nvs) = storage.as_mut() else {
            return Ok(None);
        };
        let mut buf = vec![0u8; LOG_STORAGE_LIMIT];
        let data = match nvs
            .get_raw(LOG_STORAGE_KEY, &mut buf)
            .map_err(anyhow::Error::from)?
        {
            Some(slice) => slice.to_vec(),
            None => return Ok(None),
        };
        let _ = nvs.remove(LOG_STORAGE_KEY);
        let mut text = String::from_utf8_lossy(&data).into_owned();
        if text.len() > LOG_STORAGE_LIMIT {
            text.truncate(LOG_STORAGE_LIMIT);
        }
        if text.trim().is_empty() {
            Ok(None)
        } else {
            Ok(Some(text))
        }
    }

    fn flush_on_shutdown(&self) {
        let _ = self.persist_to_nvs();
    }

    fn install_panic_hook(&self) {
        if self.panic_hook_installed.swap(true, Ordering::SeqCst) {
            return;
        }
        let default = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            if let Some(logger) = TELEMETRY_LOGGER.get() {
                logger.flush_on_shutdown();
            }
            default(info);
        }));
    }

    fn force_flush(&self) {
        if let Err(err) = self.persist_to_nvs() {
            warn!("failed to persist logs: {err:?}");
        }
    }

    fn flush_if_dirty(&self) {
        let dirty = { self.buffer.lock().unwrap().dirty };
        if dirty {
            self.force_flush();
        }
    }
}

impl log::Log for TelemetryLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        <EspLogger as log::Log>::enabled(&self.inner, metadata)
    }

    fn log(&self, record: &log::Record) {
        <EspLogger as log::Log>::log(&self.inner, record);
        self.capture(record);
    }

    fn flush(&self) {
        <EspLogger as log::Log>::flush(&self.inner);
    }
}

fn monotonic_micros() -> u64 {
    unsafe { sys::esp_timer_get_time() as u64 }
}

fn spawn_watchdog_monitor(timeout: Duration) -> Result<()> {
    let poll_interval = Duration::from_millis(WATCHDOG_MONITOR_POLL_MS);
    let margin = Duration::from_secs(WATCHDOG_MONITOR_MARGIN_SECS.min(timeout.as_secs()));
    let trigger = timeout
        .checked_sub(margin)
        .unwrap_or(Duration::from_secs(1));
    let trigger_us = trigger.as_micros() as u64;

    thread::Builder::new()
        .name("wdt-monitor".into())
        .stack_size(4096)
        .spawn(move || loop {
            thread::sleep(poll_interval);
            let last = LAST_WDT_FEED_US.load(Ordering::Relaxed);
            if last == 0 {
                continue;
            }
            let now = monotonic_micros();
            let elapsed = now.saturating_sub(last);
            if elapsed >= trigger_us && !WATCHDOG_MONITOR_TRIGGERED.swap(true, Ordering::SeqCst) {
                warn!(
                    "watchdog monitor detected stalled main task after {} us",
                    elapsed
                );
                telemetry_logger().force_flush();
                unsafe {
                    sys::esp_restart();
                }
            }
        })?;

    Ok(())
}

fn spawn_periodic_log_flush() -> Result<()> {
    let interval = Duration::from_secs(LOG_PERIODIC_FLUSH_SECS);
    thread::Builder::new()
        .name("log-flush".into())
        .stack_size(4096)
        .spawn(move || loop {
            thread::sleep(interval);
            telemetry_logger().flush_if_dirty();
        })?;
    Ok(())
}

fn main() {
    if let Err(err) = app_main() {
        error!("Fatal error: {:#}", err);
        thread::sleep(Duration::from_secs(1));
    }
}

fn app_main() -> Result<()> {
    sys::link_patches();
    let nvs_partition = EspDefaultNvsPartition::take()?;
    telemetry_logger().install(nvs_partition.clone())?;
    spawn_periodic_log_flush()?;

    let watchdog_timeout = Duration::from_secs(WATCHDOG_TIMEOUT_SECS);
    init_watchdog(watchdog_timeout)?;
    spawn_watchdog_monitor(watchdog_timeout)?;

    let reset_reason = describe_reset_reason();

    let (mut app, uid_count) = App::new(BOT_TOKEN, GIST_URL, WIFI_SSID, WIFI_PASS, nvs_partition)?;

    match telemetry_logger().take_persisted_logs() {
        Ok(Some(previous_logs)) => {
            app.send_message(
                &format!("Recovered logs from previous run:\n{previous_logs}"),
                true,
                false,
            );
        }
        Ok(None) => info!("no logs loaded from NVS"),
        Err(err) => warn!("failed to retrieve persisted logs: {err:?}"),
    }

    app.send_loaded_message(&reset_reason, uid_count);
    app.run();

    #[allow(unreachable_code)]
    Ok(())
}

struct App {
    wifi: BlockingWifi<EspWifi<'static>>,
    pn532: Pn532<I2CInterface<I2cDriver<'static>>, MonoTimer, PN532_BUFFER>,
    unlock_uart: UartDriver<'static>,
    nvs: EspDefaultNvs,
    pn_power: PinDriver<'static, Gpio19, Output>,
    boot_button: PinDriver<'static, Gpio0, Input>,
    uids: HashMap<String, String>,
    last_uid: Option<String>,
    last_seen: Instant,
    reboot_press_started: Option<Instant>,
    bot_token: &'static str,
    gist_url: &'static str,
}

impl App {
    fn new(
        bot_token: &'static str,
        gist_url: &'static str,
        wifi_ssid: &'static str,
        wifi_pass: &'static str,
        nvs_partition: EspDefaultNvsPartition,
    ) -> Result<(Self, usize)> {
        let peripherals = Peripherals::take().context("Failed to take peripherals")?;
        let pins = peripherals.pins;

        let pn_power = PinDriver::output(pins.gpio19)?;
        let mut boot_button = PinDriver::input(pins.gpio0)?;
        boot_button.set_pull(Pull::Up)?;

        let sys_loop = EspSystemEventLoop::take()?;

        let mut wifi = BlockingWifi::wrap(
            EspWifi::new(
                peripherals.modem,
                sys_loop.clone(),
                Some(nvs_partition.clone()),
            )?,
            sys_loop,
        )?;

        if let Err(e) = connect_wifi(&mut wifi, wifi_ssid, wifi_pass) {
            warn!("wifi connect fail: {e:?}");
        }

        let nvs: EspDefaultNvs = EspNvs::new(nvs_partition, NVS_NAMESPACE, true)?;

        let unlock_uart_config = uart::config::Config::new().baudrate(Hertz(9_600));
        let unlock_uart = UartDriver::new(
            peripherals.uart1,
            pins.gpio18,
            pins.gpio13,
            Option::<AnyIOPin>::None,
            Option::<AnyIOPin>::None,
            &unlock_uart_config,
        )?;
        unlock_uart.clear_rx()?;

        let i2c_config = i2c::config::Config::new()
            .baudrate(Hertz(10_000))
            .scl_enable_pullup(false)
            .sda_enable_pullup(false)
            .timeout(APBTickType::from(Duration::from_millis(10)));
        let i2c = I2cDriver::new(peripherals.i2c0, pins.gpio21, pins.gpio22, &i2c_config)?;

        let pn532 = Pn532::new(I2CInterface { i2c }, MonoTimer::default());

        let mut app = App {
            wifi,
            pn532,
            unlock_uart,
            nvs,
            pn_power,
            boot_button,
            uids: HashMap::new(),
            last_uid: None,
            last_seen: Instant::now(),
            reboot_press_started: None,
            bot_token,
            gist_url,
        };

        let count = app.refresh_uids()?;

        Ok((app, count))
    }

    fn run(&mut self) -> ! {
        self.initialize_pn532();
        let mut fail_counter = 0;
        let mut last_log = Instant::now();
        loop {
            match self.listen_once() {
                Ok(maybe_uid) => {
                    fail_counter = 0;
                    if let Some(uid) = maybe_uid {
                        self.handle_uid(uid);
                    }
                    feed_watchdog();
                    if last_log.elapsed() > Duration::from_secs(60) {
                        info!(".");
                        last_log = Instant::now();
                    }
                }
                Err(err) => {
                    fail_counter += 1;
                    warn!("listen failed: {err}");
                    if fail_counter > 10 {
                        fail_counter = 0;
                        self.send_message(&format!("listen failed: {err}"), true, true);
                        self.initialize_pn532();
                    }
                    thread::sleep(Duration::from_millis(100));
                }
            }
            self.check_manual_reboot();
        }
    }

    fn send_loaded_message(&self, reset_reason: &str, count: usize) {
        self.send_message(
            &format!("initializing done\nreset reason: {reset_reason}\nloaded {count} uids"),
            true,
            true,
        );
    }

    fn listen_once(&mut self) -> Result<Option<Vec<u8>>> {
        match self.pn532.process(
            &Request::INLIST_ONE_ISO_A_TARGET,
            48,
            LISTEN_TIMEOUT_MS.ms(),
        ) {
            Ok(response) => Ok(extract_uid(response).map(|uid| uid.to_vec())),
            Err(PnError::TimeoutAck) | Err(PnError::TimeoutResponse) => Ok(None),
            Err(err) => Err(anyhow!("PN532 error: {:?}", err)),
        }
    }

    fn handle_uid(&mut self, uid: Vec<u8>) {
        let now = Instant::now();
        let uid_hex = to_hex(&uid);

        if self
            .last_uid
            .as_ref()
            .map(|last| {
                last == &uid_hex
                    && now.duration_since(self.last_seen)
                        < Duration::from_secs(UID_REPEAT_SUPPRESSION_SECS)
            })
            .unwrap_or(false)
        {
            return;
        }

        if let Some(name) = self.uids.get(&uid_hex) {
            if let Err(err) = unlock(&self.unlock_uart) {
                warn!("failed to unlock: {err}");
            }
            self.send_message(&format!("[{name}] Authorized via NFC"), false, true);
        } else {
            self.send_message(&format!("Unknown NFC uid: {uid_hex}"), true, true);
        }

        self.last_uid = Some(uid_hex);
        self.last_seen = now;
    }

    fn check_manual_reboot(&mut self) {
        if self.boot_button.is_low() {
            match self.reboot_press_started {
                Some(start) => {
                    if start.elapsed() >= Duration::from_secs(MANUAL_REBOOT_HOLD_SECS) {
                        self.trigger_soft_reboot("BOOT button");
                    }
                }
                None => {
                    self.reboot_press_started = Some(Instant::now());
                }
            }
        } else {
            self.reboot_press_started = None;
        }
    }

    #[allow(unreachable_code)]
    fn trigger_soft_reboot(&self, reason: &str) -> ! {
        warn!("soft reboot requested: {reason}");
        self.send_message(&format!("Soft reboot requested ({reason})"), true, true);
        telemetry_logger().force_flush();
        thread::sleep(Duration::from_millis(200));
        unsafe {
            sys::esp_restart();
        }
        unreachable!("esp_restart returned unexpectedly");
    }

    fn send_message(&self, message: &str, private: bool, log: bool) {
        if log {
            info!("TG > {message}");
        }
        if let Err(err) = send_telegram(&self.bot_token, message, private, log) {
            warn!("telegram error: {err}");
        }
    }

    fn refresh_uids(&mut self) -> Result<usize> {
        match fetch_uids(&self.gist_url) {
            Ok(uids) => {
                info!("Loaded {} NFC identities from gist", uids.len());
                self.uids = uids;
                if let Err(err) = self.save_uids_cache() {
                    warn!("failed to cache keys: {err}");
                }
            }
            Err(fetch_err) => {
                warn!("Failed to fetch NFC identities: {fetch_err}");
                match self.load_uids_from_cache()? {
                    Some(cached) => {
                        self.uids = cached;
                        info!("Loaded {} NFC identities from cache", self.uids.len());
                    }
                    None => {
                        warn!("No cached NFC identities available in NVS");
                        return Err(fetch_err);
                    }
                }
            }
        }

        Ok(self.uids.len())
    }

    fn save_uids_cache(&mut self) -> Result<()> {
        let json = serde_json::to_vec(&self.uids)?;
        if json.len() > NVS_MAX_CACHE_SIZE {
            anyhow::bail!(
                "UID cache ({}) exceeds NVS buffer size {}",
                json.len(),
                NVS_MAX_CACHE_SIZE
            );
        }
        self.nvs
            .set_raw(NVS_KEYS_KEY, &json)
            .map_err(anyhow::Error::from)?;
        Ok(())
    }

    fn load_uids_from_cache(&self) -> Result<Option<HashMap<String, String>>> {
        let mut buf = vec![0u8; NVS_MAX_CACHE_SIZE];
        match self
            .nvs
            .get_raw(NVS_KEYS_KEY, &mut buf)
            .map_err(anyhow::Error::from)?
        {
            Some(data) => {
                let parsed = serde_json::from_slice(data)?;
                Ok(Some(parsed))
            }
            None => Ok(None),
        }
    }

    fn initialize_pn532(&mut self) {
        loop {
            let _ = self.pn_power.set_low();
            thread::sleep(Duration::from_millis(250));
            let _ = self.pn_power.set_high();
            thread::sleep(Duration::from_millis(250));
            match self
                .pn532
                .process(&Request::GET_FIRMWARE_VERSION, 4, 1000_u64.ms())
            {
                Ok(data) => {
                    if data.len() >= 4 {
                        info!("Found PN532 with firmware version: {}.{}", data[1], data[2]);
                    } else {
                        info!(
                            "Found PN532 (unexpected firmware response length: {})",
                            data.len()
                        );
                    }
                }
                Err(err) => {
                    warn!("Error connecting to PN532: {:?}", err);
                    thread::sleep(Duration::from_millis(PN532_RETRY_DELAY_MS));
                    continue;
                }
            }

            match self.pn532.process(
                &Request::sam_configuration(SAMMode::Normal, false),
                0,
                1000_u64.ms(),
            ) {
                Ok(_) => break,
                Err(err) => {
                    warn!("Failed to configure PN532 SAM: {:?}", err);
                    thread::sleep(Duration::from_millis(PN532_RETRY_DELAY_MS));
                }
            }
        }
    }
}

fn connect_wifi(
    wifi: &mut BlockingWifi<EspWifi<'static>>,
    ssid: &str,
    password: &str,
) -> Result<()> {
    let auth_method = if password.is_empty() {
        AuthMethod::None
    } else {
        AuthMethod::WPA2Personal
    };

    let mut client_conf = ClientConfiguration::default();
    client_conf.ssid = ssid
        .try_into()
        .map_err(|_| anyhow!("SSID '{ssid}' is too long"))?;
    client_conf.auth_method = auth_method;
    client_conf.password = password
        .try_into()
        .map_err(|_| anyhow!("Wi-Fi password is too long"))?;
    client_conf.channel = None;

    let wifi_configuration = Configuration::Client(client_conf);

    wifi.set_configuration(&wifi_configuration)?;
    wifi.start()?;
    info!("Wi-Fi started");
    wifi.connect()?;
    info!("Wi-Fi connected");
    wifi.wait_netif_up()?;
    info!("Wi-Fi netif up");
    Ok(())
}

fn send_telegram(bot_token: &str, message: &str, private: bool, log: bool) -> Result<()> {
    let mut client = HttpClient::wrap(EspHttpConnection::new(&HTTP_CONFIG)?);
    let chat_id = if private {
        TELEGRAM_PRIVATE_CHAT_ID
    } else {
        TELEGRAM_PUBLIC_CHAT_ID
    };
    let encoded_message = percent_encode(message);
    let url = format!(
        "https://api.telegram.org/bot{bot_token}/sendMessage?chat_id={chat_id}&text={encoded_message}"
    );

    let request = client.request(Method::Get, &url, &[])?;
    let mut response = request.submit()?;

    let mut body = Vec::new();
    let mut truncated = false;
    let mut buffer = [0_u8; 512];
    loop {
        match response.read(&mut buffer) {
            Ok(0) => break,
            Ok(n) => {
                let remaining = HTTP_RESPONSE_LOG_LIMIT.saturating_sub(body.len());
                if remaining == 0 {
                    truncated = true;
                    break;
                }
                let take = remaining.min(n);
                body.extend_from_slice(&buffer[..take]);
                if take < n {
                    truncated = true;
                    break;
                }
            }
            Err(err) => return Err(anyhow!("http read error: {err}")),
        }
    }

    if log {
        if truncated {
            info!("TG < {}... [truncated]", String::from_utf8_lossy(&body));
        } else {
            info!("TG < {}", String::from_utf8_lossy(&body));
        }
    }

    Ok(())
}

fn fetch_uids(url: &str) -> Result<HashMap<String, String>> {
    let mut client = HttpClient::wrap(EspHttpConnection::new(&HTTP_CONFIG)?);
    let request = client.request(Method::Get, url, &[])?;
    let mut response = request.submit()?;

    let mut body = Vec::new();
    let mut buffer = [0_u8; 512];
    loop {
        match response.read(&mut buffer) {
            Ok(0) => break,
            Ok(n) => body.extend_from_slice(&buffer[..n]),
            Err(err) => return Err(anyhow!("failed to read gist response: {err}")),
        }
    }

    let parsed: HashMap<String, String> =
        serde_json::from_slice(&body).map_err(|err| anyhow!("failed to parse gist json: {err}"))?;

    Ok(parsed)
}

fn unlock(uart: &UartDriver<'static>) -> Result<()> {
    uart.clear_rx()?;
    let written = uart.write(&[b'u'])?;
    if written != 1 {
        warn!("unexpected bytes written to unlock uart: {written}");
    }
    uart.wait_tx_done(TickType::from(Duration::from_millis(100)).into())?;
    Ok(())
}

fn init_watchdog(timeout: Duration) -> Result<()> {
    let timeout_ms = timeout.as_millis().clamp(1, u128::from(u32::MAX)) as u32;
    let core_mask = if sys::portNUM_PROCESSORS >= 32 {
        u32::MAX
    } else {
        (1u32 << sys::portNUM_PROCESSORS) - 1
    };

    let config = sys::esp_task_wdt_config_t {
        timeout_ms,
        idle_core_mask: core_mask,
        trigger_panic: true,
    };

    unsafe {
        match sys::esp!(sys::esp_task_wdt_init(&config)) {
            Ok(_) => {}
            Err(err) if err.code() == sys::ESP_ERR_INVALID_STATE => {
                sys::esp!(sys::esp_task_wdt_reconfigure(&config))?;
            }
            Err(err) => return Err(err.into()),
        }
        sys::esp!(sys::esp_task_wdt_add(ptr::null_mut()))?;
    }
    LAST_WDT_FEED_US.store(monotonic_micros(), Ordering::Relaxed);
    Ok(())
}

fn feed_watchdog() {
    unsafe {
        if let Err(err) = sys::esp!(sys::esp_task_wdt_reset()) {
            warn!("failed to feed watchdog: {:?}", err);
        }
    }
    LAST_WDT_FEED_US.store(monotonic_micros(), Ordering::Relaxed);
}

fn describe_reset_reason() -> String {
    match unsafe { sys::esp_reset_reason() } {
        sys::esp_reset_reason_t_ESP_RST_UNKNOWN => "Unknown".to_string(),
        sys::esp_reset_reason_t_ESP_RST_POWERON => "PowerOn".to_string(),
        sys::esp_reset_reason_t_ESP_RST_EXT => "External".to_string(),
        sys::esp_reset_reason_t_ESP_RST_SW => "Software".to_string(),
        sys::esp_reset_reason_t_ESP_RST_PANIC => "Panic".to_string(),
        sys::esp_reset_reason_t_ESP_RST_INT_WDT => "Interrupt WDT".to_string(),
        sys::esp_reset_reason_t_ESP_RST_TASK_WDT => "Task WDT".to_string(),
        sys::esp_reset_reason_t_ESP_RST_WDT => "Other WDT".to_string(),
        sys::esp_reset_reason_t_ESP_RST_DEEPSLEEP => "Deep Sleep".to_string(),
        sys::esp_reset_reason_t_ESP_RST_BROWNOUT => "Brownout".to_string(),
        sys::esp_reset_reason_t_ESP_RST_SDIO => "SDIO".to_string(),
        sys::esp_reset_reason_t_ESP_RST_USB => "USB".to_string(),
        sys::esp_reset_reason_t_ESP_RST_JTAG => "JTAG".to_string(),
        sys::esp_reset_reason_t_ESP_RST_EFUSE => "EFuse".to_string(),
        sys::esp_reset_reason_t_ESP_RST_PWR_GLITCH => "Power glitch".to_string(),
        sys::esp_reset_reason_t_ESP_RST_CPU_LOCKUP => "CPU lockup".to_string(),
        other => format!("code {other}"),
    }
}

fn percent_encode(input: &str) -> String {
    let mut encoded = String::with_capacity(input.len() * 3);
    for byte in input.as_bytes() {
        encoded.push('%');
        encoded.push(nibble_to_hex(byte >> 4));
        encoded.push(nibble_to_hex(byte & 0x0F));
    }
    encoded
}

fn nibble_to_hex(value: u8) -> char {
    match value {
        0..=9 => (b'0' + value) as char,
        10..=15 => (b'a' + (value - 10)) as char,
        _ => '?',
    }
}

fn to_hex(data: &[u8]) -> String {
    let mut out = String::with_capacity(data.len() * 2);
    for &byte in data {
        out.push(nibble_to_hex(byte >> 4));
        out.push(nibble_to_hex(byte & 0x0F));
    }
    out
}

fn extract_uid(response: &[u8]) -> Option<&[u8]> {
    if response.len() < 7 {
        return None;
    }
    if response[0] == 0 {
        return None;
    }
    let uid_len = response[5] as usize;
    let start: usize = 6;
    if response.len() <= start {
        return None;
    }
    let remaining = response.len() - start;
    if uid_len > remaining {
        return None;
    }
    let end = start + uid_len;
    if end > response.len() {
        return None;
    }
    Some(&response[start..end])
}

#[derive(Default)]
struct MonoTimer {
    deadline: Option<Instant>,
}

impl CountDown for MonoTimer {
    type Time = Duration;

    fn start<T>(&mut self, count: T)
    where
        T: Into<Self::Time>,
    {
        let duration = count.into();
        self.deadline = Some(Instant::now() + duration);
    }

    fn wait(&mut self) -> nb::Result<(), Infallible> {
        if let Some(deadline) = self.deadline {
            if Instant::now() >= deadline {
                Ok(())
            } else {
                Err(nb::Error::WouldBlock)
            }
        } else {
            Ok(())
        }
    }
}
