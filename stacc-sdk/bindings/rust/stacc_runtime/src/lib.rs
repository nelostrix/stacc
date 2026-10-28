//! Rust client for the STACC controller runtime API.
//!
//! Speaks the same versioned `/v1/runtime/*` JSON contract over loopback HTTP
//! as the Python client (`stacc/server/runtime_client.py`) and the C++ header
//! (`bindings/cpp/stacc_runtime.hpp`), so controllers are portable between the
//! three. No dependencies beyond `std`. Units are SI: radians, seconds,
//! newton-metres, metres.
//!
//! ```no_run
//! /* <<<RSS>>>
//! robot arm { body { shape: box; } arm main { joints: 3; reach: 600mm; } }
//! <<</RSS>>> */
//! let mut rt = stacc_runtime::Runtime::default();
//! rt.load_file(file!())?;          // the robot described in this file's RSS block
//! rt.start()?;
//! rt.command_joint("arm_main_2_joint", 0.5)?;
//! for j in rt.joints()? { println!("{} {:.3}", j.name, j.position); }
//! # Ok::<(), stacc_runtime::Error>(())
//! ```

use std::fmt;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::time::Duration;

// ─────────────────────────────── errors ───────────────────────────────

#[derive(Debug)]
pub enum Error {
    /// The daemon could not be reached or the connection failed.
    Io(std::io::Error),
    /// The daemon answered with an error (HTTP status, message).
    Daemon(u16, String),
    /// The response was not the JSON the contract describes.
    Protocol(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "stacc: {e}"),
            Error::Daemon(status, msg) => write!(f, "stacc: {msg} (HTTP {status})"),
            Error::Protocol(msg) => write!(f, "stacc: {msg}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}

pub type Result<T> = std::result::Result<T, Error>;

// ─────────────────────────────── JSON ───────────────────────────────

/// A parsed JSON value (only what the runtime contract needs).
#[derive(Debug, Clone, PartialEq)]
pub enum Json {
    Null,
    Bool(bool),
    Number(f64),
    Str(String),
    Array(Vec<Json>),
    Object(Vec<(String, Json)>),
}

static NULL: Json = Json::Null;

impl Json {
    /// Object member, or `Json::Null` when missing.
    pub fn get(&self, key: &str) -> &Json {
        match self {
            Json::Object(members) => members.iter().find(|(k, _)| k == key).map(|(_, v)| v).unwrap_or(&NULL),
            _ => &NULL,
        }
    }
    pub fn as_f64(&self) -> Option<f64> {
        if let Json::Number(n) = self { Some(*n) } else { None }
    }
    pub fn as_str(&self) -> Option<&str> {
        if let Json::Str(s) = self { Some(s) } else { None }
    }
    pub fn as_array(&self) -> &[Json] {
        if let Json::Array(a) = self { a } else { &[] }
    }

    pub fn parse(text: &str) -> Result<Json> {
        let bytes = text.as_bytes();
        let mut i = 0;
        let value = parse_value(bytes, &mut i)?;
        skip_ws(bytes, &mut i);
        if i != bytes.len() {
            return Err(Error::Protocol("trailing data in JSON response".into()));
        }
        Ok(value)
    }
}

fn skip_ws(s: &[u8], i: &mut usize) {
    while *i < s.len() && matches!(s[*i], b' ' | b'\n' | b'\r' | b'\t') {
        *i += 1;
    }
}

fn expect(s: &[u8], i: &mut usize, c: u8) -> Result<()> {
    skip_ws(s, i);
    if *i < s.len() && s[*i] == c {
        *i += 1;
        Ok(())
    } else {
        Err(Error::Protocol(format!("expected '{}' in JSON", c as char)))
    }
}

fn parse_value(s: &[u8], i: &mut usize) -> Result<Json> {
    skip_ws(s, i);
    let c = *s.get(*i).ok_or_else(|| Error::Protocol("unexpected end of JSON".into()))?;
    match c {
        b'{' => {
            *i += 1;
            let mut members = Vec::new();
            skip_ws(s, i);
            if s.get(*i) == Some(&b'}') {
                *i += 1;
                return Ok(Json::Object(members));
            }
            loop {
                skip_ws(s, i);
                let key = parse_string(s, i)?;
                expect(s, i, b':')?;
                members.push((key, parse_value(s, i)?));
                skip_ws(s, i);
                match s.get(*i) {
                    Some(b',') => *i += 1,
                    Some(b'}') => {
                        *i += 1;
                        return Ok(Json::Object(members));
                    }
                    _ => return Err(Error::Protocol("expected ',' or '}' in JSON".into())),
                }
            }
        }
        b'[' => {
            *i += 1;
            let mut items = Vec::new();
            skip_ws(s, i);
            if s.get(*i) == Some(&b']') {
                *i += 1;
                return Ok(Json::Array(items));
            }
            loop {
                items.push(parse_value(s, i)?);
                skip_ws(s, i);
                match s.get(*i) {
                    Some(b',') => *i += 1,
                    Some(b']') => {
                        *i += 1;
                        return Ok(Json::Array(items));
                    }
                    _ => return Err(Error::Protocol("expected ',' or ']' in JSON".into())),
                }
            }
        }
        b'"' => Ok(Json::Str(parse_string(s, i)?)),
        b't' if s[*i..].starts_with(b"true") => {
            *i += 4;
            Ok(Json::Bool(true))
        }
        b'f' if s[*i..].starts_with(b"false") => {
            *i += 5;
            Ok(Json::Bool(false))
        }
        b'n' if s[*i..].starts_with(b"null") => {
            *i += 4;
            Ok(Json::Null)
        }
        _ => {
            let start = *i;
            while *i < s.len() && matches!(s[*i], b'-' | b'+' | b'.' | b'e' | b'E' | b'0'..=b'9') {
                *i += 1;
            }
            std::str::from_utf8(&s[start..*i])
                .ok()
                .and_then(|t| t.parse::<f64>().ok())
                .map(Json::Number)
                .ok_or_else(|| Error::Protocol("invalid JSON value".into()))
        }
    }
}

fn parse_string(s: &[u8], i: &mut usize) -> Result<String> {
    if s.get(*i) != Some(&b'"') {
        return Err(Error::Protocol("expected string in JSON".into()));
    }
    *i += 1;
    let mut out: Vec<u8> = Vec::new();
    while *i < s.len() && s[*i] != b'"' {
        let c = s[*i];
        *i += 1;
        if c != b'\\' {
            out.push(c);
            continue;
        }
        let e = *s.get(*i).ok_or_else(|| Error::Protocol("bad escape in JSON".into()))?;
        *i += 1;
        match e {
            b'n' => out.push(b'\n'),
            b't' => out.push(b'\t'),
            b'r' => out.push(b'\r'),
            b'b' => out.push(8),
            b'f' => out.push(12),
            b'u' => {
                let hex = std::str::from_utf8(s.get(*i..*i + 4).unwrap_or_default()).unwrap_or("");
                let cp = u32::from_str_radix(hex, 16).map_err(|_| Error::Protocol("bad \\u escape".into()))?;
                *i += 4;
                let ch = char::from_u32(cp).unwrap_or('\u{FFFD}');
                let mut buf = [0u8; 4];
                out.extend_from_slice(ch.encode_utf8(&mut buf).as_bytes());
            }
            other => out.push(other),
        }
    }
    *i += 1;
    String::from_utf8(out).map_err(|_| Error::Protocol("invalid UTF-8 in JSON string".into()))
}

/// JSON string literal for `s`.
pub fn json_escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn json_numbers(values: &[f64]) -> String {
    let items: Vec<String> = values.iter().map(|v| format!("{v:?}")).collect();
    format!("[{}]", items.join(","))
}

// ─────────────────────────────── client ───────────────────────────────

#[derive(Debug, Clone)]
pub struct JointState {
    pub id: String,
    pub name: String,
    /// rad (m for prismatic joints); NaN when not measured.
    pub position: f64,
    pub velocity: f64,
    /// N·m actually applied.
    pub torque: f64,
}

pub struct Runtime {
    host: String,
    port: u16,
    timeout: Duration,
}

impl Default for Runtime {
    /// The local daemon on 127.0.0.1:8766.
    fn default() -> Self {
        Runtime::new("127.0.0.1", 8766)
    }
}

impl Runtime {
    pub fn new(host: &str, port: u16) -> Self {
        Runtime { host: host.to_string(), port, timeout: Duration::from_secs(10) }
    }

    // ── lifecycle ──

    pub fn load_source(&mut self, source: &str, format: &str) -> Result<Json> {
        self.load_source_from(source, format, None)
    }

    /// Like `load_source`; `file` is where the text came from, so `use "parts.rss:x";` resolves next to it.
    pub fn load_source_from(&mut self, source: &str, format: &str, file: Option<&str>) -> Result<Json> {
        let file_part = file.map(|f| format!(",\"file\":{}", json_escape(f))).unwrap_or_default();
        self.post(
            "/v1/runtime/load_project",
            &format!("{{\"format\":{},\"source\":{}{}}}", json_escape(format), json_escape(source), file_part),
        )
    }

    /// Load a `.rss` file, or a Rust/C++/Python file with an RSS block (pass `file!()`).
    pub fn load_file(&mut self, path: &str) -> Result<Json> {
        let source = std::fs::read_to_string(path)?;
        let format = path.rsplit_once('.').map(|(_, ext)| ext).unwrap_or("rss");
        let absolute = std::fs::canonicalize(path).ok().map(|p| p.to_string_lossy().into_owned());
        self.load_source_from(&source, format, absolute.as_deref())
    }

    pub fn start(&mut self) -> Result<Json> {
        self.post("/v1/runtime/start", "{}")
    }
    pub fn pause(&mut self) -> Result<Json> {
        self.post("/v1/runtime/pause", "{}")
    }
    pub fn resume(&mut self) -> Result<Json> {
        self.post("/v1/runtime/resume", "{}")
    }
    pub fn reset(&mut self) -> Result<Json> {
        self.post("/v1/runtime/reset", "{}")
    }
    pub fn stop(&mut self) -> Result<Json> {
        self.post("/v1/runtime/stop", "{}")
    }
    pub fn step(&mut self, count: u32) -> Result<Json> {
        self.post("/v1/runtime/step", &format!("{{\"count\":{count}}}"))
    }

    // ── state ──

    pub fn telemetry(&mut self) -> Result<Json> {
        self.get("/v1/runtime/telemetry")
    }

    pub fn joints(&mut self) -> Result<Vec<JointState>> {
        let data = self.get("/v1/runtime/joints")?;
        Ok(data
            .get("joints")
            .as_array()
            .iter()
            .map(|j| JointState {
                id: j.get("id").as_str().unwrap_or("").to_string(),
                name: j.get("name").as_str().unwrap_or("").to_string(),
                position: j.get("position").as_f64().unwrap_or(f64::NAN),
                velocity: j.get("velocity").as_f64().unwrap_or(f64::NAN),
                torque: j.get("torque").as_f64().unwrap_or(f64::NAN),
            })
            .collect())
    }

    /// Base pose [x, y, z, roll, pitch, yaw]; fixed-base robots stay at zero.
    pub fn pose(&mut self) -> Result<Vec<f64>> {
        Ok(self.telemetry()?.get("pose").as_array().iter().map(|v| v.as_f64().unwrap_or(0.0)).collect())
    }

    // ── commands ──

    pub fn command_joint(&mut self, joint: &str, angle_rad: f64) -> Result<Json> {
        self.post(
            "/v1/runtime/joints/command",
            &format!("{{\"joint\":{},\"angle_rad\":{angle_rad:?}}}", json_escape(joint)),
        )
    }
    pub fn command_positions(&mut self, positions_rad: &[f64]) -> Result<Json> {
        self.post("/v1/runtime/joints/command", &format!("{{\"positions\":{}}}", json_numbers(positions_rad)))
    }
    /// Direct joint torques (N·m); clears any position command.
    pub fn command_torques(&mut self, torques_nm: &[f64]) -> Result<Json> {
        self.post("/v1/runtime/joints/command", &format!("{{\"torques\":{}}}", json_numbers(torques_nm)))
    }
    /// Rotor thrust (N) in the robot's rotor order; clamped to each rotor's maximum.
    pub fn command_thrusts(&mut self, thrusts_n: &[f64]) -> Result<Json> {
        self.post("/v1/runtime/joints/command", &format!("{{\"thrusts\":{}}}", json_numbers(thrusts_n)))
    }
    // ── shared-memory control channel (the data plane) ──

    /// Enable the channel (`"realtime"` or `"lockstep"`) and open it.
    pub fn enable_shm(&mut self, mode: &str, timeout_ms: f64) -> Result<ControlChannel> {
        let status = self.post(
            "/v1/runtime/shm",
            &format!("{{\"enable\":true,\"mode\":{},\"timeout_ms\":{timeout_ms:?}}}", json_escape(mode)),
        )?;
        let path = status.get("path").as_str().ok_or_else(|| Error::Protocol("no channel path".into()))?;
        ControlChannel::open(path)
    }
    pub fn shm_status(&mut self) -> Result<Json> {
        self.get("/v1/runtime/shm")
    }
    pub fn disable_shm(&mut self) -> Result<Json> {
        self.post("/v1/runtime/shm", "{\"enable\":false}")
    }

    pub fn gains(&mut self, kp: f64, kd: f64) -> Result<Json> {
        self.post("/v1/runtime/control/gains", &format!("{{\"kp\":{kp:?},\"kd\":{kd:?}}}"))
    }

    // ── transport ──

    fn get(&mut self, path: &str) -> Result<Json> {
        self.request("GET", path, "")
    }
    fn post(&mut self, path: &str, body: &str) -> Result<Json> {
        self.request("POST", path, body)
    }

    fn request(&mut self, method: &str, path: &str, body: &str) -> Result<Json> {
        let mut stream = TcpStream::connect((self.host.as_str(), self.port))?;
        stream.set_read_timeout(Some(self.timeout))?;
        stream.set_write_timeout(Some(self.timeout))?;
        let mut req = format!("{method} {path} HTTP/1.0\r\nHost: {}\r\n", self.host);
        if method == "POST" {
            req.push_str(&format!("Content-Type: application/json\r\nContent-Length: {}\r\n", body.len()));
        }
        req.push_str("\r\n");
        req.push_str(body);
        stream.write_all(req.as_bytes())?;

        let mut resp = String::new();
        stream.read_to_string(&mut resp)?;
        let (head, payload) =
            resp.split_once("\r\n\r\n").ok_or_else(|| Error::Protocol("malformed HTTP response".into()))?;
        let status: u16 = head.split_whitespace().nth(1).and_then(|s| s.parse().ok()).unwrap_or(0);
        let data = Json::parse(payload)?;
        if status >= 400 || !matches!(data.get("error"), Json::Null) {
            let msg = data.get("error").as_str().map(str::to_string).unwrap_or_else(|| format!("HTTP {status}"));
            return Err(Error::Daemon(status, msg));
        }
        Ok(data)
    }
}

// ─────────────────────── shared-memory control channel ───────────────────────

/// One frame of the runtime's state block.
#[derive(Debug, Clone, Default)]
pub struct ChannelState {
    pub step: u64,
    pub sim_time: f64,
    /// x, y, z, qw, qx, qy, qz of the base, world frame.
    pub pose: [f64; 7],
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub torque: Vec<f64>,
    pub thrust: Vec<f64>,
}

/// Shared-memory control channel (layout v1; see stacc/server/rt_channel.py).
///
/// std-only: positioned reads/writes on the `/dev/shm` file, which on Linux
/// tmpfs operate on the same pages the runtime has mapped. Each block is a
/// seqlock (odd sequence = being written), so reads never see a torn frame.
pub struct ControlChannel {
    file: std::fs::File,
    n_joints: usize,
    n_thrusters: usize,
    lockstep: bool,
    state_off: u64,
    cmd_off: u64,
    next_id: u64,
}

const CHANNEL_MAGIC: u32 = 0x5452_5453; // 'STRT'
const CHANNEL_VERSION: u32 = 1;
const KIND_POSITION: u32 = 1;
const KIND_TORQUE: u32 = 2;

impl ControlChannel {
    pub fn open(path: &str) -> Result<Self> {
        use std::os::unix::fs::FileExt;
        let file = std::fs::OpenOptions::new().read(true).write(true).open(path)?;
        let mut h = [0u8; 40];
        file.read_exact_at(&mut h, 0)?;
        let u32_at = |i: usize| u32::from_le_bytes(h[i..i + 4].try_into().unwrap());
        let u64_at = |i: usize| u64::from_le_bytes(h[i..i + 8].try_into().unwrap());
        if u32_at(0) != CHANNEL_MAGIC {
            return Err(Error::Protocol(format!("{path} is not a STACC control channel")));
        }
        if u32_at(4) != CHANNEL_VERSION {
            return Err(Error::Protocol("unsupported control channel layout".into()));
        }
        Ok(ControlChannel {
            file,
            n_joints: u32_at(8) as usize,
            n_thrusters: u32_at(12) as usize,
            lockstep: u32_at(16) == 1,
            state_off: u64_at(24),
            cmd_off: u64_at(32),
            next_id: 0,
        })
    }

    pub fn n_joints(&self) -> usize {
        self.n_joints
    }
    pub fn n_thrusters(&self) -> usize {
        self.n_thrusters
    }
    pub fn lockstep(&self) -> bool {
        self.lockstep
    }

    fn read_u64(&self, off: u64) -> Result<u64> {
        use std::os::unix::fs::FileExt;
        let mut b = [0u8; 8];
        self.file.read_exact_at(&mut b, off)?;
        Ok(u64::from_le_bytes(b))
    }

    /// Latest consistent state, or None if the writer stayed mid-update.
    pub fn read_state(&self) -> Result<Option<ChannelState>> {
        use std::os::unix::fs::FileExt;
        let n = 7 + 3 * self.n_joints + self.n_thrusters;
        let mut buf = vec![0u8; 24 + n * 8];
        for _ in 0..10_000 {
            let s1 = self.read_u64(self.state_off)?;
            if s1 & 1 == 1 {
                continue;
            }
            self.file.read_exact_at(&mut buf, self.state_off)?;
            if self.read_u64(self.state_off)? != s1 || u64::from_le_bytes(buf[0..8].try_into().unwrap()) != s1 {
                continue;
            }
            let f = |i: usize| f64::from_le_bytes(buf[24 + i * 8..32 + i * 8].try_into().unwrap());
            let (nj, nt) = (self.n_joints, self.n_thrusters);
            let mut pose = [0.0; 7];
            for (k, p) in pose.iter_mut().enumerate() {
                *p = f(k);
            }
            return Ok(Some(ChannelState {
                step: u64::from_le_bytes(buf[8..16].try_into().unwrap()),
                sim_time: f64::from_le_bytes(buf[16..24].try_into().unwrap()),
                pose,
                position: (0..nj).map(|i| f(7 + i)).collect(),
                velocity: (0..nj).map(|i| f(7 + nj + i)).collect(),
                torque: (0..nj).map(|i| f(7 + 2 * nj + i)).collect(),
                thrust: (0..nt).map(|i| f(7 + 3 * nj + i)).collect(),
            }));
        }
        Ok(None)
    }

    /// Spin until the runtime publishes a step newer than `after_step` (None: any).
    pub fn wait_state(&self, after_step: Option<u64>, timeout: Duration) -> Result<Option<ChannelState>> {
        let deadline = std::time::Instant::now() + timeout;
        while std::time::Instant::now() < deadline {
            if let Some(s) = self.read_state()? {
                if after_step.map_or(true, |a| s.step > a) {
                    return Ok(Some(s));
                }
            }
        }
        Ok(None)
    }

    pub fn command_positions(&mut self, rad: &[f64], answers_step: u64, thrust: Option<&[f64]>) -> Result<u64> {
        self.write_command(KIND_POSITION, rad, answers_step, thrust)
    }
    pub fn command_torques(&mut self, nm: &[f64], answers_step: u64, thrust: Option<&[f64]>) -> Result<u64> {
        self.write_command(KIND_TORQUE, nm, answers_step, thrust)
    }

    fn write_command(&mut self, kind: u32, values: &[f64], answers_step: u64, thrust: Option<&[f64]>) -> Result<u64> {
        use std::os::unix::fs::FileExt;
        if values.len() != self.n_joints {
            return Err(Error::Protocol(format!("expected {} joint values, got {}", self.n_joints, values.len())));
        }
        if let Some(t) = thrust {
            if t.len() != self.n_thrusters {
                return Err(Error::Protocol(format!("expected {} thrust values, got {}", self.n_thrusters, t.len())));
            }
        }
        let id = self.next_id.max(self.read_u64(self.cmd_off + 8)?) + 1;
        self.next_id = id;
        let start = self.read_u64(self.cmd_off)? + 1; // odd: writing
        self.file.write_all_at(&start.to_le_bytes(), self.cmd_off)?;
        let mut body = Vec::with_capacity(24 + 8 * (self.n_joints + self.n_thrusters));
        body.extend_from_slice(&id.to_le_bytes());
        body.extend_from_slice(&answers_step.to_le_bytes());
        body.extend_from_slice(&kind.to_le_bytes());
        body.extend_from_slice(&(thrust.is_some() as u32).to_le_bytes());
        for v in values {
            body.extend_from_slice(&v.to_le_bytes());
        }
        for i in 0..self.n_thrusters {
            body.extend_from_slice(&thrust.map_or(0.0, |t| t[i]).to_le_bytes());
        }
        self.file.write_all_at(&body, self.cmd_off + 8)?;
        self.file.write_all_at(&(start + 1).to_le_bytes(), self.cmd_off)?; // even: consistent
        Ok(id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_runtime_payload() {
        let j = Json::parse(r#"{"joints": [{"id": "J1", "name": "a", "position": -0.5, "torque": null}], "ok": true}"#).unwrap();
        let joint = &j.get("joints").as_array()[0];
        assert_eq!(joint.get("name").as_str(), Some("a"));
        assert_eq!(joint.get("position").as_f64(), Some(-0.5));
        assert_eq!(joint.get("torque"), &Json::Null);
        assert_eq!(j.get("ok"), &Json::Bool(true));
    }

    #[test]
    fn escapes_source_text() {
        assert_eq!(json_escape("a\"b\\c\nd"), r#""a\"b\\c\nd""#);
    }
}
