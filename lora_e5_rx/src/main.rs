//! LoRa-E5 Mini — last two serial/LoRa lines + decoded SSDV JPEGs.
//!
//! Optional CLI: `lora_e5_rx COM5` pre-fills the port field.
//!
//! Each **complete** SSDV image is saved under `./ssdv_received/`. Throttled live preview while
//! receiving; toolbar shows serial read buffer size and approximate UI event queue depth.

mod radio;
mod ssdv_native;

use eframe::egui;
use radio::{Event, JpegUpdate, RfConfig, run_worker, SERIAL_READER_CAP_BYTES};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread::JoinHandle;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

const SAVE_DIR: &str = "ssdv_received";

static SAVE_SEQ: AtomicU64 = AtomicU64::new(0);

/// One visual line for the log panel: no multi-line wrap (long `+TEST: RX` hex stays single row).
fn log_one_line(s: &str) -> &str {
    s.lines().next().unwrap_or("")
}

fn main() -> eframe::Result<()> {
    let prefill_port = std::env::args().nth(1);

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([980.0, 700.0])
            .with_title("LoRa-E5 Mini — SSDV RX"),
        ..Default::default()
    };

    eframe::run_native(
        "LoRa-E5 RX",
        options,
        Box::new(move |cc| Ok(Box::new(LoraApp::new(cc, prefill_port)))),
    )
}

struct LoraApp {
    port: String,
    baud: String,
    freq_mhz: String,
    sf: String,
    bw: String,
    /// Rolling buffer: older line, then newest (each new log line pushes the previous one up).
    lora_msg_prev: String,
    lora_msg_latest: String,
    log_rx: Receiver<Event>,
    log_tx: Sender<Event>,
    running: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
    debug_timing: bool,
    /// Second-to-last **complete** JPEG bytes (left panel).
    jpeg_previous: Option<Vec<u8>>,
    /// Last **complete** JPEG bytes (kept in sync after each EOI; right panel live during RX).
    jpeg_latest: Option<Vec<u8>>,
    tex_previous: Option<egui::TextureHandle>,
    tex_latest: Option<egui::TextureHandle>,
    info_previous: String,
    info_latest: String,
    /// SSDV `image_id` from last `JpegUpdate` (promote Last when this changes).
    last_rx_image_id: Option<u8>,
    /// Worker increments per `Event` sent; UI subtracts per drained batch (backlog estimate).
    ui_queue_depth: Arc<AtomicUsize>,
}

impl LoraApp {
    fn new(_cc: &eframe::CreationContext<'_>, prefill_port: Option<String>) -> Self {
        let (log_tx, log_rx) = mpsc::channel();
        let ui_queue_depth = Arc::new(AtomicUsize::new(0));
        Self {
            port: prefill_port.unwrap_or_default(),
            baud: "115200".into(),
            freq_mhz: "869.4".into(),
            sf: "7".into(),
            bw: "500".into(),
            lora_msg_prev: "—".into(),
            lora_msg_latest: "—".into(),
            log_rx,
            log_tx,
            running: Arc::new(AtomicBool::new(false)),
            worker: None,
            debug_timing: false,
            jpeg_previous: None,
            jpeg_latest: None,
            tex_previous: None,
            tex_latest: None,
            info_previous: "—".into(),
            info_latest: "No image yet".into(),
            last_rx_image_id: None,
            ui_queue_depth,
        }
    }

    fn push_lora_log(&mut self, s: String) {
        self.lora_msg_prev = std::mem::replace(&mut self.lora_msg_latest, s);
    }

    fn disconnect(&mut self) {
        self.running.store(false, Ordering::SeqCst);
        if let Some(h) = self.worker.take() {
            let _ = h.join();
        }
        self.ui_queue_depth.store(0, Ordering::Relaxed);
    }

    fn connect(&mut self) -> Result<(), String> {
        if self.worker.is_some() {
            return Ok(());
        }
        let port = self.port.trim().to_string();
        if port.is_empty() {
            return Err("Set a serial port (e.g. COM5)".into());
        }
        let baud: u32 = self
            .baud
            .trim()
            .parse()
            .map_err(|_| "Baud must be a number".to_string())?;
        let freq_mhz: f32 = self
            .freq_mhz
            .trim()
            .parse()
            .map_err(|_| "Frequency must be a number (MHz)".to_string())?;
        let sf: u8 = self
            .sf
            .trim()
            .parse()
            .map_err(|_| "SF must be 7–12".to_string())?;
        let bw: u32 = self
            .bw
            .trim()
            .parse()
            .map_err(|_| "BW must be 125, 250, or 500".to_string())?;

        let rf = RfConfig {
            freq_mhz,
            sf,
            bw,
        };

        self.running.store(true, Ordering::SeqCst);
        let run = self.running.clone();
        let tx = self.log_tx.clone();
        let qd = self.ui_queue_depth.clone();
        let port_clone = port.clone();
        let dbg = self.debug_timing;

        let h = std::thread::spawn(move || {
            run_worker(port_clone, baud, rf, tx, run, dbg, qd);
        });
        self.worker = Some(h);
        self.last_rx_image_id = None;
        self.push_lora_log(format!("--- connect {port} @ {baud} ---"));
        Ok(())
    }

    fn drain_log(&mut self, ctx: &egui::Context) {
        const MAX_EVENTS_PER_FRAME: usize = 8192;
        let batch: Vec<Event> = self.log_rx.try_iter().take(MAX_EVENTS_PER_FRAME).collect();
        let n = batch.len();
        for ev in batch {
            match ev {
                Event::Log(line) => self.push_lora_log(line),
                Event::Jpeg(j) => self.apply_jpeg(ctx, j),
            }
        }
        if n > 0 {
            self.ui_queue_depth.fetch_sub(n, Ordering::Relaxed);
            ctx.request_repaint();
        }
    }

    fn jpeg_to_color(jpeg_bytes: &[u8]) -> Result<(egui::ColorImage, [usize; 2]), image::ImageError> {
        let img = image::load_from_memory(jpeg_bytes)?;
        let rgba = img.to_rgba8();
        let size = [rgba.width() as usize, rgba.height() as usize];
        let color = egui::ColorImage::from_rgba_unmultiplied(size, rgba.as_raw());
        Ok((color, size))
    }

    fn set_texture(
        ctx: &egui::Context,
        slot: &mut Option<egui::TextureHandle>,
        id: &str,
        color: egui::ColorImage,
    ) {
        let opts = egui::TextureOptions::LINEAR;
        match slot.as_mut() {
            None => *slot = Some(ctx.load_texture(id, color, opts)),
            Some(tex) => tex.set(color, opts),
        }
    }

    fn apply_jpeg(&mut self, ctx: &egui::Context, j: JpegUpdate) {
        let JpegUpdate {
            frame,
            jpeg_for_save,
            packets,
            complete,
            image_id,
        } = j;

        if self.last_rx_image_id != Some(image_id) {
            if self.last_rx_image_id.is_some() {
                self.jpeg_previous = self.jpeg_latest.take();
                if let Some(ref prev_bytes) = self.jpeg_previous {
                    match Self::jpeg_to_color(prev_bytes) {
                        Ok((prev_color, prev_size)) => {
                            Self::set_texture(ctx, &mut self.tex_previous, "ssdv_previous", prev_color);
                            self.info_previous = format!(
                                "{}×{} — {} B",
                                prev_size[0], prev_size[1], prev_bytes.len()
                            );
                        }
                        Err(e) => {
                            self.push_lora_log(format!("ERROR: Last JPEG decode: {e}"));
                            self.tex_previous = None;
                            self.info_previous = "—".into();
                        }
                    }
                } else {
                    self.tex_previous = None;
                    self.info_previous = "—".into();
                }
            }
            self.last_rx_image_id = Some(image_id);
        }

        let mut size = [0_usize, 0_usize];
        if let Some((rgba, width, height)) = &frame {
            let expected = (*width as usize)
                .saturating_mul(*height as usize)
                .saturating_mul(4);
            if *width == 0 || *height == 0 || rgba.len() != expected {
                self.push_lora_log(format!(
                    "ERROR: bad RGBA frame {}×{} (got {} B, want {})",
                    width,
                    height,
                    rgba.len(),
                    expected
                ));
            } else {
                size = [*width as usize, *height as usize];
                let color = egui::ColorImage::from_rgba_unmultiplied(size, rgba);
                Self::set_texture(ctx, &mut self.tex_latest, "ssdv_latest", color);
            }
        }

        if complete {
            let save_len = jpeg_for_save.len();
            if save_len == 0 {
                self.push_lora_log(format!(
                    "WARN: EOI image id {image_id} — 0 JPEG bytes (not saved)"
                ));
            } else if let Err(e) = std::fs::create_dir_all(SAVE_DIR) {
                self.push_lora_log(format!("ERROR: create {SAVE_DIR}: {e}"));
            } else {
                let ts = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .map(|d| d.as_secs())
                    .unwrap_or(0);
                let seq = SAVE_SEQ.fetch_add(1, Ordering::Relaxed);
                let path = format!("{SAVE_DIR}/ssdv_{ts}_{seq}_id{image_id}.jpg");
                match std::fs::write(&path, &jpeg_for_save) {
                    Ok(()) => self.push_lora_log(format!("Saved full image {path} ({save_len}) B")),
                    Err(e) => self.push_lora_log(format!("ERROR: save {path}: {e}")),
                }
            }

            self.jpeg_latest = if save_len == 0 {
                None
            } else {
                Some(jpeg_for_save)
            };

            if size[0] > 0 && size[1] > 0 {
                self.info_latest = format!(
                    "{}×{} — {packets} SSDV pkts — id {image_id}",
                    size[0], size[1]
                );
            } else {
                self.info_latest = format!(
                    "No preview — {packets} pkts — id {image_id} — JPEG saved"
                );
            }
        } else if let Some((_, w, h)) = &frame {
            self.info_latest = format!(
                "Receiving… {}×{} — {packets} pkts (id {image_id}) — gray = gaps",
                w, h
            );
        } else {
            self.info_latest =
                format!("Receiving… {packets} pkts (id {image_id})");
        }
    }

    fn show_image_column(ui: &mut egui::Ui, label: &str, info: &str, tex: Option<&egui::TextureHandle>) {
        ui.vertical(|ui| {
            ui.label(egui::RichText::new(label).strong());
            ui.label(egui::RichText::new(info).small());
            if let Some(tex) = tex {
                let avail = ui.available_size();
                let mut sz = tex.size_vec2();
                if sz.x > 0.0 && sz.y > 0.0 {
                    let max_h = 360.0;
                    let scale = (avail.x / sz.x).min(max_h / sz.y.max(1.0)).max(0.05).min(1.0);
                    sz *= scale;
                    ui.add(
                        egui::Image::new(egui::load::SizedTexture::from_handle(tex)).max_size(sz),
                    );
                }
            } else {
                ui.label(egui::RichText::new("(none)").weak());
            }
        });
    }
}

impl eframe::App for LoraApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.drain_log(ctx);

        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.label("Port");
                ui.add(egui::TextEdit::singleline(&mut self.port).desired_width(120.0));
                ui.label("Baud");
                ui.add(egui::TextEdit::singleline(&mut self.baud).desired_width(80.0));
                ui.label("MHz");
                ui.add(egui::TextEdit::singleline(&mut self.freq_mhz).desired_width(56.0));
                ui.label("SF");
                ui.add(egui::TextEdit::singleline(&mut self.sf).desired_width(28.0));
                ui.label("BW");
                ui.add(egui::TextEdit::singleline(&mut self.bw).desired_width(40.0));

                if self.worker.is_some() {
                    if ui.button("Disconnect").clicked() {
                        self.disconnect();
                    }
                } else if ui.button("Connect").clicked() {
                    if let Err(e) = self.connect() {
                        self.push_lora_log(format!("ERROR: {e}"));
                    }
                }

                ui.checkbox(&mut self.debug_timing, "Debug timing");
                ui.separator();
                if ui.button("Clear log lines").clicked() {
                    self.lora_msg_prev = "—".into();
                    self.lora_msg_latest = "—".into();
                }
                let q = self.ui_queue_depth.load(Ordering::Relaxed);
                ui.label(
                    egui::RichText::new(format!(
                        "Serial buf {} KiB · UI queue ~{q}",
                        SERIAL_READER_CAP_BYTES / 1024
                    ))
                    .small()
                    .weak(),
                );
                ui.label(egui::RichText::new(format!("Saves → ./{SAVE_DIR}/")).small().weak());
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.vertical(|ui| {
                ui.label(egui::RichText::new("Last 2 serial / AT / LoRa lines (newest at bottom)").weak());
                egui::Frame::default()
                    .fill(ui.visuals().extreme_bg_color)
                    .inner_margin(egui::Margin::same(8.0))
                    .rounding(4.0)
                    .show(ui, |ui| {
                        ui.scope(|ui| {
                            ui.style_mut().override_font_id = Some(egui::FontId::monospace(12.0));
                            ui.vertical(|ui| {
                                ui.label(egui::RichText::new("Previous:").small().weak());
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(log_one_line(&self.lora_msg_prev))
                                            .monospace(),
                                    )
                                    .truncate(),
                                );
                                ui.add_space(6.0);
                                ui.label(egui::RichText::new("Latest:").small().weak());
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(log_one_line(&self.lora_msg_latest))
                                            .monospace(),
                                    )
                                    .truncate(),
                                );
                            });
                        });
                    });

                ui.add_space(12.0);
                ui.label(egui::RichText::new("Images").strong());
                ui.separator();

                ui.columns(2, |cols| {
                    cols[0].vertical(|ui| {
                        LoraApp::show_image_column(
                            ui,
                            "Last",
                            &self.info_previous,
                            self.tex_previous.as_ref(),
                        );
                    });
                    cols[1].vertical(|ui| {
                        LoraApp::show_image_column(
                            ui,
                            "Current",
                            &self.info_latest,
                            self.tex_latest.as_ref(),
                        );
                    });
                });

                if self.worker.is_none() {
                    ui.add_space(8.0);
                    ui.label(egui::RichText::new("Connect to receive.").weak());
                }
            });
        });

        if self.worker.is_some() {
            // Worker can push events faster than vsync; repaint soon so the channel stays near empty.
            ctx.request_repaint_after(Duration::from_millis(4));
        }
    }

    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        self.disconnect();
    }
}
