/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

//! Opens the first USB3 Vision camera, receives image buffers, and logs FPS.
//!
//! Usage:
//! ```sh
//! cargo run --example buffer_framerate --features=libusb -- [preview_bytes]
//! ```

use std::{env, error::Error, time::Instant};

use cameleon::{
    genapi::{GenApiCtxt, ParamsCtxt},
    u3v::enumerate_cameras,
    DeviceControl, StreamError,
};

const DEFAULT_PREVIEW_BYTES: usize = 16;
const STREAM_CHANNEL_CAPACITY: usize = 3;
const MAX_CONSECUTIVE_TIMEOUTS: usize = 30;

fn main() -> Result<(), Box<dyn Error>> {
    let preview_bytes = parse_arg(1, DEFAULT_PREVIEW_BYTES);

    let mut cameras = enumerate_cameras()?;
    if cameras.is_empty() {
        println!("no USB3 Vision camera found");
        return Ok(());
    }

    let mut camera = cameras.remove(0);
    println!("opening camera: {:?}", camera.info());
    camera.open()?;
    camera.load_context()?;
    configure_for_continuous_acquisition(&mut camera.params_ctxt()?)?;

    let payload_rx = camera.start_streaming(STREAM_CHANNEL_CAPACITY)?;
    println!("streaming until Ctrl+C; previewing {preview_bytes} bytes per buffer");

    let stream_started = Instant::now();
    let mut last_frame = stream_started;
    let mut received_frames = 0;
    let mut consecutive_timeouts = 0;

    loop {
        let payload = match payload_rx.recv_blocking() {
            Ok(payload) => {
                consecutive_timeouts = 0;
                payload
            }
            Err(StreamError::Timeout) => {
                consecutive_timeouts += 1;
                println!(
                    "stream timeout while waiting for frame {}; continuing ({consecutive_timeouts}/{MAX_CONSECUTIVE_TIMEOUTS})",
                    received_frames + 1
                );

                if consecutive_timeouts >= MAX_CONSECUTIVE_TIMEOUTS {
                    println!(
                        "stopping after {MAX_CONSECUTIVE_TIMEOUTS} consecutive stream timeouts"
                    );
                    break;
                }

                continue;
            }
            Err(err) => {
                camera.close().ok();
                return Err(err.into());
            }
        };

        received_frames += 1;
        let now = Instant::now();
        let elapsed = now.duration_since(stream_started).as_secs_f64();
        let frame_delta = now.duration_since(last_frame).as_secs_f64();
        last_frame = now;

        let fps = if elapsed > 0.0 {
            received_frames as f64 / elapsed
        } else {
            0.0
        };
        let instantaneous_fps = if frame_delta > 0.0 {
            1.0 / frame_delta
        } else {
            0.0
        };

        let buffer = payload.payload();
        let image_len = payload.image().map_or(0, |image| image.len());
        let preview = hex_preview(buffer, preview_bytes);

        println!(
            "#{received_frames:04} block_id={} timestamp={:?} payload_type={:?} payload_bytes={} image_bytes={} fps={fps:.2} inst_fps={instantaneous_fps:.2} preview=[{}]",
            payload.id(),
            payload.timestamp(),
            payload.payload_type(),
            buffer.len(),
            image_len,
            preview,
        );

        if let Some(image_info) = payload.image_info() {
            println!("       image_info={image_info:?}");
        }

        payload_rx.send_back(payload);
    }

    camera.close()?;
    Ok(())
}

fn parse_arg(index: usize, default: usize) -> usize {
    env::args()
        .nth(index)
        .and_then(|arg| arg.parse().ok())
        .unwrap_or(default)
}

fn hex_preview(buffer: &[u8], preview_bytes: usize) -> String {
    buffer
        .iter()
        .take(preview_bytes)
        .map(|byte| format!("{byte:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn configure_for_continuous_acquisition<Ctrl, Ctxt>(
    params_ctxt: &mut ParamsCtxt<Ctrl, Ctxt>,
) -> Result<(), Box<dyn Error>>
where
    Ctrl: DeviceControl,
    Ctxt: GenApiCtxt,
{
    let Some(acquisition_mode) = params_ctxt
        .node("AcquisitionMode")
        .and_then(|node| node.as_enumeration(params_ctxt))
    else {
        println!("AcquisitionMode node not found; streaming with current camera settings");
        return Ok(());
    };

    if acquisition_mode.is_writable(params_ctxt)? {
        acquisition_mode.set_entry_by_symbolic(params_ctxt, "Continuous")?;
        println!("set AcquisitionMode=Continuous");
    } else if acquisition_mode.is_readable(params_ctxt)? {
        let current = acquisition_mode.current_entry(params_ctxt)?;
        println!(
            "AcquisitionMode is read-only; current value is {}",
            current.symbolic(params_ctxt)
        );
    } else {
        println!("AcquisitionMode is not writable; streaming with current camera settings");
    }

    Ok(())
}
