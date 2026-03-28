//! LoRa-E5 AT TEST-mode receive loop (blocking serial).

use anyhow::{anyhow, Context, Result};
use std::collections::BTreeMap;
use std::io::{BufRead, BufReader, ErrorKind};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use crate::ssdv_native::{packet_ok, preview_replay_ordered};

/// Serial `read` timeout after port open / successful AT (ms).
const SERIAL_DEFAULT_TIMEOUT_MS: u64 = 2500;
/// Max wait for the **first** UART line after `AT+TEST=RXLRPKT` (idle between SSDV bursts).
const LORA_PACKET_WAIT_SECS: u64 = 60;
/// After the first UART line in a session, wait this long for each following line before re-arming
/// with `RXLRPKT`. Must cover LoRa packet airtime + E5 UART jitter + host scheduling (Windows).
const RX_BURST_LINE_TIMEOUT_MS: u64 = 3000;
/// Packet-count-only UI tick when live preview decode is skipped or throttled.
const PARTIAL_UI_EVERY: usize = 8;
/// Try live SSDV→JPEG preview every N packets (plus first), gated by `PREVIEW_MIN_GAP_MS`.
const PREVIEW_EVERY: usize = 10;
const PREVIEW_MIN_GAP_MS: u64 = 120;
/// `BufReader` capacity on the serial port (shown in UI).
pub const SERIAL_READER_CAP_BYTES: usize = 128 * 1024;
/// Wall time budget to drain trailing UART lines before the next RXLRPKT.
const SLURP_TRAILING_CAP_MS: u64 = 500;
/// Per-read timeout inside slurp (ms).
const SLURP_READ_TIMEOUT_MS: u64 = 150;
/// `expect_contains`: timeout per `read_line` and total wait for the needle.
const AT_LINE_TIMEOUT_SECS: u64 = 5;
const AT_RESPONSE_TOTAL_SECS: u64 = 15;

#[derive(Clone, Debug)]
pub struct RfConfig {
    pub freq_mhz: f32,
    pub sf: u8,
    pub bw: u32,
}

#[derive(Debug)]
pub struct JpegUpdate {
    /// RGBA8 + size when decode succeeded; `None` if preview/EOI JPEG failed `image` decode (file may still save).
    pub frame: Option<(Vec<u8>, u32, u32)>,
    /// Full SSDV output JPEG when `complete` (always sent on EOI if replay produced bytes).
    pub jpeg_for_save: Vec<u8>,
    pub packets: usize,
    pub complete: bool,
    pub image_id: u8,
}

fn jpeg_bytes_to_rgba(jpeg: &[u8]) -> Result<(Vec<u8>, u32, u32)> {
    let img = image::load_from_memory(jpeg)?;
    let rgba = img.to_rgba8();
    let (w, h) = rgba.dimensions();
    Ok((rgba.into_raw(), w, h))
}

#[derive(Debug)]
pub enum Event {
    Log(String),
    Jpeg(JpegUpdate),
}

fn emit(tx: &Sender<Event>, q: &AtomicUsize, ev: Event) {
    q.fetch_add(1, Ordering::Relaxed);
    let _ = tx.send(ev);
}

fn log(tx: &Sender<Event>, q: &AtomicUsize, s: impl Into<String>) {
    emit(tx, q, Event::Log(s.into()));
}

pub fn run_worker(
    port_name: String,
    baud: u32,
    rf: RfConfig,
    tx: Sender<Event>,
    running: Arc<AtomicBool>,
    debug_timing: bool,
    ui_queue_depth: Arc<AtomicUsize>,
) {
    let q = ui_queue_depth.as_ref();
    if let Err(e) = run_inner(&port_name, baud, rf, &tx, &running, debug_timing, q) {
        log(&tx, q, format!("ERROR: {e:#}"));
    }
    log(&tx, q, "--- disconnected ---");
}

fn run_inner(
    port_name: &str,
    baud: u32,
    rf: RfConfig,
    tx: &Sender<Event>,
    running: &Arc<AtomicBool>,
    debug: bool,
    q: &AtomicUsize,
) -> Result<()> {
    const SSDV_PKT: i32 = 255;
    let mut ssdv_stash: BTreeMap<u16, Vec<u8>> = BTreeMap::new();
    // Last SSDV image_id in stash; clear stash when it changes (new picture).
    let mut ssdv_stash_image_id: Option<u8> = None;
    let mut preview_work = vec![0u8; 1024 * 1024];
    let mut last_live_preview = Instant::now()
        .checked_sub(Duration::from_millis(500))
        .unwrap_or_else(Instant::now);
    if !(7..=12).contains(&rf.sf) {
        anyhow::bail!("SF must be 7..=12");
    }
    if ![125, 250, 500].contains(&rf.bw) {
        anyhow::bail!("BW must be 125, 250, or 500");
    }

    let mut port = serialport::new(port_name, baud)
        .timeout(Duration::from_millis(SERIAL_DEFAULT_TIMEOUT_MS))
        .data_bits(serialport::DataBits::Eight)
        .flow_control(serialport::FlowControl::None)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .open()
        .with_context(|| format!("open serial port {port_name}"))?;

    log(tx, q, format!("Opened {port_name} @ {baud} baud"));
    log(
        tx,
        q,
        format!(
            "RF: {} MHz SF{} BW{} (match ESP32 / LORA_* env on old CLI)",
            rf.freq_mhz, rf.sf, rf.bw
        ),
    );

    let _ = drain_reader(port.as_mut(), Duration::from_millis(50));

    if let Err(e) = expect_contains(port.as_mut(), "AT\r\n", "+AT: OK", "AT", tx, q) {
        if baud == 115200 {
            log(
                tx,
                q,
                "No AT at 115200, trying bootstrap at 9600 -> set UART 115200...",
            );
            drop(port);
            bootstrap_uart_115200(port_name, tx, q)?;
            port = serialport::new(port_name, 115200)
                .timeout(Duration::from_millis(SERIAL_DEFAULT_TIMEOUT_MS))
                .data_bits(serialport::DataBits::Eight)
                .flow_control(serialport::FlowControl::None)
                .parity(serialport::Parity::None)
                .stop_bits(serialport::StopBits::One)
                .open()
                .with_context(|| format!("re-open serial port {port_name} @115200"))?;
            let _ = drain_reader(port.as_mut(), Duration::from_millis(100));
            expect_contains(port.as_mut(), "AT\r\n", "+AT: OK", "AT(115200)", tx, q)?;
            log(tx, q, "UART now active at 115200");
        } else {
            return Err(e);
        }
    }
    expect_contains(
        port.as_mut(),
        "AT+MODE=TEST\r\n",
        "+MODE: TEST",
        "AT+MODE=TEST",
        tx,
        q,
    )?;

    let sf_str = format!("SF{}", rf.sf);
    let rfcfg = format!(
        "AT+TEST=RFCFG,{},{},{},8,15,14,ON,OFF,OFF\r\n",
        fmt_freq(rf.freq_mhz),
        sf_str,
        rf.bw
    );
    expect_contains(port.as_mut(), &rfcfg, "+TEST: RFCFG", "AT+TEST=RFCFG", tx, q)?;

    log(tx, q, "Listening… (Disconnect to stop.)");
    if debug {
        log(
            tx,
            q,
            "[DBG] One AT+TEST=RXLRPKT per burst; then read many +TEST: RX lines until UART idle",
        );
    }

    let mut reader = BufReader::with_capacity(SERIAL_READER_CAP_BYTES, port);

    let mut session: u64 = 0;
    let mut pkt_seq: u64 = 0;
    while running.load(Ordering::SeqCst) {
        session += 1;
        // Consume any leftover lines from the previous +TEST response (avoid stale data / misalignment).
        slurp_trailing_lines(
            &mut reader,
            Duration::from_millis(SLURP_TRAILING_CAP_MS),
        )?;

        if debug {
            log(
                tx,
                q,
                format!("[DBG] session#{session} → AT+TEST=RXLRPKT (one arm for many packets)"),
            );
        }
        let t_session = Instant::now();
        write_all_cmd(reader.get_mut().as_mut(), "AT+TEST=RXLRPKT\r\n")?;

        let mut first_line = true;
        // Long wait until first UART line; shorter waits between following lines in this session.
        let mut line_timeout = Duration::from_secs(LORA_PACKET_WAIT_SECS);

        loop {
            if !running.load(Ordering::SeqCst) {
                return Ok(());
            }

            reader.get_mut().set_timeout(line_timeout)?;

            let mut line = String::new();
            let read_res = reader.read_line(&mut line);

            match read_res {
                Ok(0) => return Err(anyhow!("serial EOF")),
                Ok(_) => {}
                Err(e) => {
                    if e.kind() == std::io::ErrorKind::TimedOut {
                        if debug {
                            log(
                                tx,
                                q,
                                format!(
                                    "[DBG] session#{session} UART idle after {:?} → new RXLRPKT next",
                                    t_session.elapsed()
                                ),
                            );
                        } else {
                            log(tx, q, "(RX burst idle, RXLRPKT)".to_string());
                        }
                        break;
                    }
                    return Err(e).context("read_line");
                }
            }

            let t = line.trim_end();
            if !t.is_empty() {
                line_timeout = Duration::from_millis(RX_BURST_LINE_TIMEOUT_MS);
            }
            if t.is_empty() {
                continue;
            }
            if debug && first_line {
                log(
                    tx,
                    q,
                    format!(
                        "[DBG] session#{session} first line +{:?}: {}",
                        t_session.elapsed(),
                        t
                    ),
                );
                first_line = false;
            } else if first_line {
                first_line = false;
            }

            let payload_hex = extract_lora_hex_payload(t);
            if debug || payload_hex.is_none() {
                log(tx, q, t.to_string());
            }

            if let Some(rest) = t.strip_prefix("+TEST: LEN:") {
                if debug {
                    log(
                        tx,
                        q,
                        format!(
                            "[DBG] session#{session} meta +{:?}: LEN line ({})",
                            t_session.elapsed(),
                            rest
                        ),
                    );
                }
            }

            if let Some(hex_part) = payload_hex {
                match decode_hex(&hex_part) {
                    Ok(bytes) => {
                        pkt_seq += 1;
                        if debug {
                            let air_hdr: String = bytes
                                .iter()
                                .take(4)
                                .map(|b| format!("{b:02X}"))
                                .collect::<Vec<_>>()
                                .join(" ");
                            let preview: String = bytes
                                .iter()
                                .take(12)
                                .map(|b| format!("{b:02X}"))
                                .collect::<Vec<_>>()
                                .join(" ");
                            log(
                                tx,
                                q,
                                format!(
                                    "[DBG] PKT#{pkt_seq} session#{session} LoRa {} B, air[0..4]={air_hdr} (NOFEC ~55 67, NORMAL ~55 66) | first12: {} … (+{:?})",
                                    bytes.len(),
                                    preview,
                                    t_session.elapsed()
                                ),
                            );
                        }
                        if packet_ok(&bytes, SSDV_PKT) {
                            let image_id = bytes[6];
                            let packet_id =
                                u16::from_be_bytes([bytes[7], bytes[8]]);
                            let eoi = (bytes[11] & 0x04) != 0;

                            match ssdv_stash_image_id {
                                None => {
                                    ssdv_stash_image_id = Some(image_id);
                                }
                                Some(prev) if prev != image_id => {
                                    ssdv_stash.clear();
                                    ssdv_stash_image_id = Some(image_id);
                                    last_live_preview = Instant::now()
                                        .checked_sub(Duration::from_millis(400))
                                        .unwrap_or_else(Instant::now);
                                }
                                _ => {}
                            }
                            ssdv_stash.insert(packet_id, bytes);

                            let ordered: Vec<&Vec<u8>> = ssdv_stash
                                .values()
                                .filter(|p| p.len() == SSDV_PKT as usize)
                                .collect();

                            if ordered.is_empty() {
                                if debug {
                                    log(
                                        tx,
                                        q,
                                        format!(
                                            "SSDV pkt image={} packet={} eoi={} (stash empty after insert?)",
                                            image_id, packet_id, eoi
                                        ),
                                    );
                                }
                            } else {
                                let first_pid =
                                    u16::from_be_bytes([ordered[0][7], ordered[0][8]]);
                                if first_pid != 0 {
                                    if debug {
                                        log(
                                            tx,
                                            q,
                                            format!(
                                                "SSDV pkt image={} packet={} eoi={} (waiting for packet 0; stash has {} pkts)",
                                                image_id, packet_id, eoi, ordered.len()
                                            ),
                                        );
                                    }
                                } else if eoi {
                                    let slices: Vec<&[u8]> =
                                        ordered.iter().map(|v| v.as_slice()).collect();
                                    match preview_replay_ordered(&slices, SSDV_PKT, &mut preview_work)
                                    {
                                        Ok(Some(mut jpeg)) => {
                                            if debug {
                                                log(
                                                    tx,
                                                    q,
                                                    format!(
                                                        "SSDV EOI image={} pkt={} | {} pkts → {} B JPEG",
                                                        image_id,
                                                        packet_id,
                                                        ordered.len(),
                                                        jpeg.len()
                                                    ),
                                                );
                                            }
                                            let frame = jpeg_bytes_to_rgba(&jpeg).ok();
                                            if frame.is_none() {
                                                log(
                                                    tx,
                                                    q,
                                                    format!(
                                                        "SSDV RGBA decode failed (image={} pkt={}); JPEG file still saved",
                                                        image_id, packet_id
                                                    ),
                                                );
                                            }
                                            let jpeg_for_save = std::mem::take(&mut jpeg);
                                            emit(
                                                tx,
                                                q,
                                                Event::Jpeg(JpegUpdate {
                                                    frame,
                                                    jpeg_for_save,
                                                    packets: ordered.len(),
                                                    complete: true,
                                                    image_id,
                                                }),
                                            );
                                        }
                                        Ok(None) => {
                                            if debug {
                                                log(
                                                    tx,
                                                    q,
                                                    format!(
                                                        "SSDV EOI image={} pkt={} (preview replay skipped)",
                                                        image_id, packet_id
                                                    ),
                                                );
                                            }
                                        }
                                        Err(e) => {
                                            log(tx, q, format!("SSDV preview replay error: {e:#}"))
                                        }
                                    }
                                } else {
                                    let min_gap = Duration::from_millis(PREVIEW_MIN_GAP_MS);
                                    let want_preview = ordered.len() == 1
                                        || (ordered.len() % PREVIEW_EVERY == 0);
                                    let mut sent_frame = false;
                                    if want_preview && last_live_preview.elapsed() >= min_gap {
                                        let slices: Vec<&[u8]> =
                                            ordered.iter().map(|v| v.as_slice()).collect();
                                        match preview_replay_ordered(
                                            &slices,
                                            SSDV_PKT,
                                            &mut preview_work,
                                        ) {
                                            Ok(Some(jpeg)) => {
                                                last_live_preview = Instant::now();
                                                let frame = jpeg_bytes_to_rgba(&jpeg).ok();
                                                emit(
                                                    tx,
                                                    q,
                                                    Event::Jpeg(JpegUpdate {
                                                        frame,
                                                        jpeg_for_save: Vec::new(),
                                                        packets: ordered.len(),
                                                        complete: false,
                                                        image_id,
                                                    }),
                                                );
                                                sent_frame = true;
                                            }
                                            Ok(None) | Err(_) => {}
                                        }
                                    }
                                    if !sent_frame
                                        && (ordered.len() == 1
                                            || (ordered.len() % PARTIAL_UI_EVERY == 0))
                                    {
                                        emit(
                                            tx,
                                            q,
                                            Event::Jpeg(JpegUpdate {
                                                frame: None,
                                                jpeg_for_save: Vec::new(),
                                                packets: ordered.len(),
                                                complete: false,
                                                image_id,
                                            }),
                                        );
                                    }
                                }
                            }
                        } else if debug {
                            let air_hdr: String = bytes
                                .iter()
                                .take(4)
                                .map(|b| format!("{b:02X}"))
                                .collect::<Vec<_>>()
                                .join(" ");
                            log(
                                tx,
                                q,
                                format!(
                                    "[DBG] PKT#{pkt_seq} session#{session} not valid SSDV (sync/type/CRC/FEC); air[0..4]={air_hdr} (+{:?})",
                                    t_session.elapsed()
                                ),
                            );
                        }
                    }
                    Err(e) => log(tx, q, format!(">>> bad hex: {e}")),
                }
                if debug {
                    log(
                        tx,
                        q,
                        format!(
                            "[DBG] PKT#{pkt_seq} session#{session} line done in {:?}",
                            t_session.elapsed()
                        ),
                    );
                }
                continue;
            }

            if t.contains("ERROR") || t.contains("+AT: ERROR") {
                return Err(anyhow!("module error: {t}"));
            }
        }
    }

    Ok(())
}

/// Read and discard newline-terminated lines until idle for one read timeout or `cap` elapsed.
/// Restores the port timeout that was active on entry (caller should set long timeout again if needed).
fn slurp_trailing_lines(
    reader: &mut BufReader<Box<dyn serialport::SerialPort>>,
    cap: Duration,
) -> Result<()> {
    let prev = reader.get_mut().timeout();
    reader
        .get_mut()
        .set_timeout(Duration::from_millis(SLURP_READ_TIMEOUT_MS))?;
    let start = Instant::now();
    let mut line = String::new();
    let res = (|| {
        while start.elapsed() < cap {
            line.clear();
            match reader.read_line(&mut line) {
                Ok(0) => break,
                Ok(_) => continue,
                Err(e) if e.kind() == ErrorKind::TimedOut => break,
                Err(e) => return Err(e).context("slurp trailing UART"),
            }
        }
        Ok(())
    })();
    let _ = reader.get_mut().set_timeout(prev);
    res
}

fn bootstrap_uart_115200(port_name: &str, tx: &Sender<Event>, q: &AtomicUsize) -> Result<()> {
    let mut p = serialport::new(port_name, 9600)
        .timeout(Duration::from_millis(SERIAL_DEFAULT_TIMEOUT_MS))
        .data_bits(serialport::DataBits::Eight)
        .flow_control(serialport::FlowControl::None)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .open()
        .with_context(|| format!("open serial port {port_name} @9600 for bootstrap"))?;

    let _ = drain_reader(p.as_mut(), Duration::from_millis(80));
    expect_contains(p.as_mut(), "AT\r\n", "+AT: OK", "AT(9600 bootstrap)", tx, q)?;
    // LoRa-E5 applies UART baud after reset.
    expect_contains(
        p.as_mut(),
        "AT+UART=BR,115200\r\n",
        "+UART",
        "AT+UART=BR,115200",
        tx,
        q,
    )?;
    // Best effort reset. If unsupported, user can power-cycle manually.
    let _ = write_all_cmd(p.as_mut(), "AT+RESET\r\n");
    thread::sleep(Duration::from_millis(1200));
    Ok(())
}

fn fmt_freq(mhz: f32) -> String {
    if (mhz - mhz.round()).abs() < 0.0001 {
        format!("{}", mhz as i32)
    } else {
        format!("{mhz}")
    }
}

fn write_all_cmd(port: &mut dyn serialport::SerialPort, cmd: &str) -> Result<()> {
    port.write_all(cmd.as_bytes()).context("write AT command")?;
    port.flush().context("flush")?;
    thread::sleep(Duration::from_millis(30));
    Ok(())
}

fn drain_reader(port: &mut dyn serialport::SerialPort, total: Duration) -> Result<()> {
    let old = port.timeout();
    port.set_timeout(Duration::from_millis(20))?;
    let mut buf = [0u8; 256];
    let start = std::time::Instant::now();
    while start.elapsed() < total {
        match port.read(&mut buf) {
            Ok(0) => break,
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => {
                port.set_timeout(old)?;
                return Err(e).context("drain read");
            }
        }
    }
    port.set_timeout(old)?;
    Ok(())
}

fn expect_contains(
    port: &mut dyn serialport::SerialPort,
    cmd: &str,
    needle: &str,
    label: &str,
    tx: &Sender<Event>,
    q: &AtomicUsize,
) -> Result<()> {
    write_all_cmd(port, cmd)?;
    port.set_timeout(Duration::from_secs(AT_LINE_TIMEOUT_SECS))?;
    let mut acc = String::new();
    let start = std::time::Instant::now();
    let mut reader = BufReader::new(&mut *port);
    while start.elapsed() < Duration::from_secs(AT_RESPONSE_TOTAL_SECS) {
        let mut line = String::new();
        let read_res = reader.read_line(&mut line);
        match read_res {
            Ok(0) => break,
            Ok(_) => {
                let t = line.trim_end();
                if !t.is_empty() {
                    log(tx, q, t.to_string());
                }
                acc.push_str(&line);
                if acc.contains(needle) {
                    port.set_timeout(Duration::from_millis(SERIAL_DEFAULT_TIMEOUT_MS))?;
                    return Ok(());
                }
                if acc.contains("ERROR") {
                    return Err(anyhow!("{label} failed (ERROR in response)"));
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => return Err(e).context("read response"),
        }
    }
    Err(anyhow!(
        "timeout waiting for '{needle}' after {label} (AT firmware supports AT+MODE=TEST?)"
    ))
}

/// Hex from `+TEST: RX "…"` (case-insensitive), or a standalone all-hex line (255 B → 510 chars).
fn extract_lora_hex_payload(line: &str) -> Option<String> {
    let t = line.trim();
    let lower = t.to_ascii_lowercase();
    let key = "+test: rx";
    if let Some(i) = lower.find(key) {
        let rest = &t[i + key.len()..];
        let start_quote = rest.find('"')?;
        let inner = &rest[start_quote + 1..];
        let end_quote = inner.find('"')?;
        return Some(inner[..end_quote].to_string());
    }
    let compact: String = t.chars().filter(|c| !c.is_whitespace()).collect();
    if compact.len() == 510 && compact.chars().all(|c| c.is_ascii_hexdigit()) {
        return Some(compact);
    }
    None
}

fn decode_hex(s: &str) -> Result<Vec<u8>> {
    let hex: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if hex.len() % 2 != 0 {
        return Err(anyhow!("odd hex length"));
    }
    (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16))
        .collect::<std::result::Result<Vec<_>, _>>()
        .map_err(|_| anyhow!("invalid hex"))
}
