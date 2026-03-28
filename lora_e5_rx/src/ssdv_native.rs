use anyhow::{Result, anyhow};
use std::ffi::c_int;
use std::os::raw::c_uchar;

unsafe extern "C" {
    fn ssdv_rx_packet_ok(packet: *const c_uchar, packet_len: usize, pkt_size: c_int) -> c_int;
    fn ssdv_rx_preview_replay(
        packets: *const *const c_uchar,
        num_packets: usize,
        pkt_size: c_int,
        out_jpeg: *mut c_uchar,
        out_cap: usize,
        out_len: *mut usize,
    ) -> c_int;
}

pub fn packet_ok(packet: &[u8], pkt_size: i32) -> bool {
    unsafe { ssdv_rx_packet_ok(packet.as_ptr(), packet.len(), pkt_size) != 0 }
}

/// Packets sorted by ascending SSDV packet ID; first must be ID 0. Gaps use decoder gray-fill.
pub fn preview_replay_ordered(
    packets_sorted_by_id: &[&[u8]],
    pkt_size: i32,
    work: &mut [u8],
) -> Result<Option<Vec<u8>>> {
    if packets_sorted_by_id.is_empty() {
        return Ok(None);
    }
    let ptrs: Vec<*const c_uchar> = packets_sorted_by_id
        .iter()
        .map(|p| p.as_ptr())
        .collect();
    let mut out_len: usize = 0;
    let rc = unsafe {
        ssdv_rx_preview_replay(
            ptrs.as_ptr(),
            ptrs.len(),
            pkt_size,
            work.as_mut_ptr(),
            work.len(),
            &mut out_len,
        )
    };
    match rc {
        0 => Ok(Some(work[..out_len].to_vec())),
        -2 | -4 | -5 => Ok(None),
        n => Err(anyhow!("ssdv_rx_preview_replay failed ({n})")),
    }
}
