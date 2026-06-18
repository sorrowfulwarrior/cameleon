/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

//! Opens the first USB3 Vision camera, receives image buffers, and logs FPS.
//!
//! Usage:
//! ```sh
//! cargo run --example buffer_framerate --features=libusb -- [target_fps] [preview_bytes] [exposure_us]
//! ```

use std::{collections::VecDeque, env, error::Error, time::Instant};

use cameleon::{
    genapi::{GenApiCtxt, ParamsCtxt},
    u3v::enumerate_cameras,
    DeviceControl, StreamError,
};

const DEFAULT_TARGET_FPS: f64 = 160.0;
const DEFAULT_PREVIEW_BYTES: usize = 16;
const DEFAULT_EXPOSURE_US: f64 = 1000.0;
const STREAM_CHANNEL_CAPACITY: usize = 3;
const MAX_CONSECUTIVE_TIMEOUTS: usize = 30;
const ROLLING_FPS_FRAMES: usize = 60;

fn main() -> Result<(), Box<dyn Error>> {
    let target_fps = parse_arg(1, DEFAULT_TARGET_FPS);
    let preview_bytes = parse_arg(2, DEFAULT_PREVIEW_BYTES);
    let exposure_us = parse_arg(3, DEFAULT_EXPOSURE_US);

    let mut cameras = enumerate_cameras()?;
    if cameras.is_empty() {
        println!("no USB3 Vision camera found");
        return Ok(());
    }

    let mut camera = cameras.remove(0);
    println!("opening camera: {:?}", camera.info());
    camera.open()?;
    camera.load_context()?;
    configure_camera(&mut camera.params_ctxt()?, target_fps, exposure_us)?;

    let payload_rx = camera.start_streaming(STREAM_CHANNEL_CAPACITY)?;
    println!(
        "requested {target_fps:.2} FPS; streaming until Ctrl+C; previewing {preview_bytes} bytes per buffer"
    );

    let stream_started = Instant::now();
    let mut last_frame = stream_started;
    let mut frame_times = VecDeque::with_capacity(ROLLING_FPS_FRAMES);
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
        frame_times.push_back(now);
        if frame_times.len() > ROLLING_FPS_FRAMES {
            frame_times.pop_front();
        }

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
        let rolling_fps = rolling_fps(&frame_times);

        let buffer = payload.payload();
        let image_len = payload.image().map_or(0, |image| image.len());
        let preview = hex_preview(buffer, preview_bytes);

        println!(
            "#{received_frames:04} block_id={} timestamp={:?} payload_type={:?} payload_bytes={} image_bytes={} avg_fps={fps:.2} rolling_fps={rolling_fps:.2} inst_fps={instantaneous_fps:.2} target_fps={target_fps:.2} preview=[{}]",
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

fn parse_arg<T>(index: usize, default: T) -> T
where
    T: std::str::FromStr,
{
    env::args()
        .nth(index)
        .and_then(|arg| arg.parse().ok())
        .unwrap_or(default)
}

fn rolling_fps(frame_times: &VecDeque<Instant>) -> f64 {
    let Some(first) = frame_times.front() else {
        return 0.0;
    };
    let Some(last) = frame_times.back() else {
        return 0.0;
    };

    let intervals = frame_times.len().saturating_sub(1);
    let elapsed = last.duration_since(*first).as_secs_f64();
    if intervals > 0 && elapsed > 0.0 {
        intervals as f64 / elapsed
    } else {
        0.0
    }
}

fn hex_preview(buffer: &[u8], preview_bytes: usize) -> String {
    buffer
        .iter()
        .take(preview_bytes)
        .map(|byte| format!("{byte:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn configure_camera<Ctrl, Ctxt>(
    params_ctxt: &mut ParamsCtxt<Ctrl, Ctxt>,
    target_fps: f64,
    exposure_us: f64,
) -> Result<(), Box<dyn Error>>
where
    Ctrl: DeviceControl,
    Ctxt: GenApiCtxt,
{
    configure_for_continuous_acquisition(params_ctxt)?;
    disable_device_link_throughput_limit(params_ctxt)?;
    disable_exposure_auto(params_ctxt)?;
    set_exposure_time(params_ctxt, exposure_us)?;
    enable_frame_rate_control(params_ctxt)?;
    set_frame_rate(params_ctxt, target_fps)?;
    print_resulting_frame_rate(params_ctxt)?;
    Ok(())
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

fn disable_device_link_throughput_limit<Ctrl, Ctxt>(
    params_ctxt: &mut ParamsCtxt<Ctrl, Ctxt>,
) -> Result<(), Box<dyn Error>>
where
    Ctrl: DeviceControl,
    Ctxt: GenApiCtxt,
{
    let Some(throughput_limit_mode) = params_ctxt
        .node("DeviceLinkThroughputLimitMode")
        .and_then(|node| node.as_enumeration(params_ctxt))
    else {
        println!("DeviceLinkThroughputLimitMode node not found");
        return Ok(());
    };

    if throughput_limit_mode.is_writable(params_ctxt)? {
        throughput_limit_mode.set_entry_by_symbolic(params_ctxt, "Off")?;
        println!("set DeviceLinkThroughputLimitMode=Off");
    } else if throughput_limit_mode.is_readable(params_ctxt)? {
        println!(
            "DeviceLinkThroughputLimitMode is read-only; current value is {}",
            throughput_limit_mode
                .current_entry(params_ctxt)?
                .symbolic(params_ctxt)
        );
    } else {
        println!("DeviceLinkThroughputLimitMode is not writable");
    }

    Ok(())
}

fn enable_frame_rate_control<Ctrl, Ctxt>(
    params_ctxt: &mut ParamsCtxt<Ctrl, Ctxt>,
) -> Result<(), Box<dyn Error>>
where
    Ctrl: DeviceControl,
    Ctxt: GenApiCtxt,
{
    for node_name in ["AcquisitionFrameRateEnable", "AcquisitionFrameRateEnabled"] {
        let Some(enable_node) = params_ctxt
            .node(node_name)
            .and_then(|node| node.as_boolean(params_ctxt))
        else {
            continue;
        };

        if enable_node.is_writable(params_ctxt)? {
            enable_node.set_value(params_ctxt, true)?;
            println!("set {node_name}=true");
        } else if enable_node.is_readable(params_ctxt)? {
            println!(
                "{node_name} is read-only; current value is {}",
                enable_node.value(params_ctxt)?
            );
        } else {
            println!("{node_name} is not writable");
        }
        return Ok(());
    }

    println!("frame-rate enable node not found; trying to set frame rate directly");
    Ok(())
}

fn set_frame_rate<Ctrl, Ctxt>(
    params_ctxt: &mut ParamsCtxt<Ctrl, Ctxt>,
    target_fps: f64,
) -> Result<(), Box<dyn Error>>
where
    Ctrl: DeviceControl,
    Ctxt: GenApiCtxt,
{
    for node_name in ["AcquisitionFrameRate", "AcquisitionFrameRateAbs"] {
        let Some(frame_rate_node) = params_ctxt
            .node(node_name)
            .and_then(|node| node.as_float(params_ctxt))
        else {
            continue;
        };

        if !frame_rate_node.is_writable(params_ctxt)? {
            if frame_rate_node.is_readable(params_ctxt)? {
                println!(
                    "{node_name} is read-only; current value is {:.2}",
                    frame_rate_node.value(params_ctxt)?
                );
            } else {
                println!("{node_name} is not writable");
            }
            return Ok(());
        }

        let min = frame_rate_node.min(params_ctxt)?;
        let max = frame_rate_node.max(params_ctxt)?;
        let requested = target_fps.clamp(min, max);
        if (requested - target_fps).abs() > f64::EPSILON {
            println!(
                "requested FPS {target_fps:.2} is outside {node_name} range [{min:.2}, {max:.2}]; using {requested:.2}"
            );
        }

        frame_rate_node.set_value(params_ctxt, requested)?;
        println!(
            "set {node_name}={:.2}; camera reports {:.2}",
            requested,
            frame_rate_node.value(params_ctxt)?
        );
        return Ok(());
    }

    println!("no AcquisitionFrameRate node found; camera frame rate was not changed");
    Ok(())
}

fn disable_exposure_auto<Ctrl, Ctxt>(
    params_ctxt: &mut ParamsCtxt<Ctrl, Ctxt>,
) -> Result<(), Box<dyn Error>>
where
    Ctrl: DeviceControl,
    Ctxt: GenApiCtxt,
{
    let Some(exposure_auto) = params_ctxt
        .node("ExposureAuto")
        .and_then(|node| node.as_enumeration(params_ctxt))
    else {
        println!("ExposureAuto node not found");
        return Ok(());
    };

    if exposure_auto.is_writable(params_ctxt)? {
        exposure_auto.set_entry_by_symbolic(params_ctxt, "Off")?;
        println!("set ExposureAuto=Off");
    } else if exposure_auto.is_readable(params_ctxt)? {
        println!(
            "ExposureAuto is read-only; current value is {}",
            exposure_auto
                .current_entry(params_ctxt)?
                .symbolic(params_ctxt)
        );
    } else {
        println!("ExposureAuto is not writable");
    }

    Ok(())
}

fn set_exposure_time<Ctrl, Ctxt>(
    params_ctxt: &mut ParamsCtxt<Ctrl, Ctxt>,
    exposure_us: f64,
) -> Result<(), Box<dyn Error>>
where
    Ctrl: DeviceControl,
    Ctxt: GenApiCtxt,
{
    let Some(exposure_node) = params_ctxt
        .node("ExposureTime")
        .and_then(|node| node.as_float(params_ctxt))
    else {
        println!("ExposureTime node not found");
        return Ok(());
    };

    if !exposure_node.is_writable(params_ctxt)? {
        if exposure_node.is_readable(params_ctxt)? {
            println!(
                "ExposureTime is read-only; current value is {:.2} us",
                exposure_node.value(params_ctxt)?
            );
        } else {
            println!("ExposureTime is not writable");
        }
        return Ok(());
    }

    let min = exposure_node.min(params_ctxt)?;
    let max = exposure_node.max(params_ctxt)?;
    let requested = exposure_us.clamp(min, max);
    if (requested - exposure_us).abs() > f64::EPSILON {
        println!(
            "requested exposure {exposure_us:.2} us is outside ExposureTime range [{min:.2}, {max:.2}]; using {requested:.2} us"
        );
    }

    exposure_node.set_value(params_ctxt, requested)?;
    println!(
        "set ExposureTime={:.2} us; camera reports {:.2} us",
        requested,
        exposure_node.value(params_ctxt)?
    );
    Ok(())
}

fn print_resulting_frame_rate<Ctrl, Ctxt>(
    params_ctxt: &mut ParamsCtxt<Ctrl, Ctxt>,
) -> Result<(), Box<dyn Error>>
where
    Ctrl: DeviceControl,
    Ctxt: GenApiCtxt,
{
    for node_name in ["ResultingFrameRate", "ResultingFrameRateAbs"] {
        let Some(frame_rate_node) = params_ctxt
            .node(node_name)
            .and_then(|node| node.as_float(params_ctxt))
        else {
            continue;
        };

        if frame_rate_node.is_readable(params_ctxt)? {
            println!(
                "{node_name} reports {:.2} FPS",
                frame_rate_node.value(params_ctxt)?
            );
        } else {
            println!("{node_name} is not readable");
        }
        return Ok(());
    }

    println!("ResultingFrameRate node not found");
    Ok(())
}
