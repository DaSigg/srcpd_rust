//! srcpd Implementierung
//! Lizenz: GPL V3
//! Daniel Sigg
//!
//! 25.03.23 Basisversion

use configparser::ini::Ini;
use log::{error, info, warn};
use nix::{
  libc::{SIGHUP, SIGINT, SIGQUIT, SIGTERM},
  unistd::{fork, ForkResult::Parent},
};
use signal_hook::iterator::Signals;
use srcp_server_types::{SRCPMessage, SRCPMessageDevice, SRCPMessageID, SRCPMessageType};
use std::{
  cell::RefCell,
  collections::HashMap,
  env, fs, process,
  rc::Rc,
  sync::mpsc::{self, Sender},
  thread,
  time::Duration,
};

use crate::{srcp_server_ddl::DDL, srcp_server_s88::S88, srcp_server_types::Message};

mod srcp;
mod srcp_devices_ddl;
mod srcp_devices_ddl_ga;
mod srcp_devices_ddl_gl;
mod srcp_devices_ddl_power;
mod srcp_protocol_ddl;
mod srcp_protocol_ddl_dcc;
mod srcp_protocol_ddl_mfx;
mod srcp_protocol_ddl_mm;
mod srcp_server_ddl;
mod srcp_server_s88;
mod srcp_server_types;

/// PID Filename
const PID_FILE: &str = "/var/run/srcpd.pid";

/// Liefert alle vorhandenen SRCP Servertypen zurück
fn get_alle_srcp_server() -> Vec<Rc<RefCell<dyn srcp_server_types::SRCPServer>>> {
  vec![
    Rc::new(RefCell::new(S88::new())),
    Rc::new(RefCell::new(DDL::new())),
  ]
}

///Kommandozeilenparameter
#[derive(Debug)]
struct CmdLineConfig {
  //Zu verwendendes Configfile, Default ist /etc/srcpd_rust.conf
  config_file: String,
  //fork() ja/nein. Default: ja
  fork: bool,
}

impl CmdLineConfig {
  /// Liefert die aus den Kommandozeilen Argumenten gelesen Konfiguration zurück.
  /// Err mit Fehlertext -> Hilfetextausgabe, Programmabruch
  /// -? -> Hilfetext, Programmabruch
  /// -n -> No fork()
  /// -f configfile -> zu verwendendes Configfile
  /// # Arguments
  /// * args - Kommandozeilenargumente
  fn parse_cmd_line(mut args: impl Iterator<Item = String>) -> Result<CmdLineConfig, String> {
    //Ignoriere arg[0], eigener Pfad
    args.next();
    //Defaults
    let mut cmd_line_config = CmdLineConfig {
      config_file: format!("/etc/{}.conf", env!("CARGO_PKG_NAME")).to_string(),
      fork: true,
    };
    loop {
      match args.next() {
        Some(val) => match val.as_str() {
          "-?" => {
            return Err("".to_string());
          }
          "-n" => {
            cmd_line_config.fork = false;
          }
          "-f" => {
            cmd_line_config.config_file = match args.next() {
              Some(val) => val,
              _ => return Err("-f ohne Configfile".to_string()),
            }
          }
          _ => {
            return Err(format!("Unbekannter Parameter {val}"));
          }
        },
        None => break,
      }
    }
    Ok(cmd_line_config)
  }
}

///Main
fn main() {
  env::set_var("RUST_BACKTRACE", "1");
  env::set_var("RUST_LOG", "INFO");
  env_logger::builder().format_timestamp_millis().init();
  if let Err(msg) = start(env::args()) {
    error!("Start Error: {}", msg);
  }
}

/// Power Off für alle vorhandenen Busse wenn Programm terminiert wird
/// # Arguments
/// * all_cmd_tx - alle Sender für alle vorhandene SRCP Server
fn terminate_poweroff(all_cmd_tx: HashMap<usize, Sender<Message>>) {
  let mut signals = Signals::new(&[SIGTERM, SIGINT, SIGHUP, SIGQUIT]).unwrap();
  for _ in signals.forever() {
    //Allen SRCP Server Power Off senden
    for (bus, server) in all_cmd_tx {
      if server
        .send(Message::new_srcpmessage(SRCPMessage::new(
          Some(0),
          bus,
          SRCPMessageID::Command {
            msg_type: (SRCPMessageType::SET),
          },
          SRCPMessageDevice::Power,
          vec!["OFF".to_string()],
        )))
        .is_err()
      {
        warn!("Terminate Send Power Off fail");
      }
    }
    //Kurze Pause damit alles ausgeschaltet werden kann
    thread::sleep(Duration::from_millis(200));
    process::exit(0);
  }
}

///PID File schreiben
/// # Arguments
/// * pid - Aktuelle, zu schreibende PID
fn write_pidfile(pid: i32) {
  if fs::write(&PID_FILE, pid.to_string()).is_err() {
    warn!("PID konnte nicht gespeichert werden.");
  }
}

///PID File löschen
fn del_pidfile() {
  fs::remove_file(PID_FILE).unwrap_or(());
}

///Start srcpd_rust
/// # Arguments
/// * args - Kommandozeilenargumente
fn start(args: impl Iterator<Item = String>) -> Result<(), String> {
  println!(
    "{} V{} {}",
    env!("CARGO_PKG_NAME"),
    env!("CARGO_PKG_VERSION"),
    env!("CARGO_PKG_HOMEPAGE")
  );
  let cmd_line_config = match CmdLineConfig::parse_cmd_line(args) {
    Ok(v) => v,
    Err(message) => {
      println!("Aufruf: {} [-n] [-f configfile]", env!("CARGO_PKG_NAME"));
      println!("-n No fork()");
      println!("-f configfile Verwende configfile");
      println!("{message}");
      return Ok(());
    }
  };
  //fork() wenn notwendig
  if cmd_line_config.fork {
    info!("fork()");
    let pid = unsafe { fork() };
    match pid.expect("Fork Failed: Unable to create child process!") {
      Parent { child: child_pid } => {
        //PID File schreiben
        write_pidfile(child_pid.into());
        return Ok(());
      }
      _ => (),
    }
  }
  //Configfile lesen
  let mut config = Ini::new();
  let config_file_values = config.load(&cmd_line_config.config_file).expect(
    format!(
      "Configfile {} kann nicht gelesen werden",
      cmd_line_config.config_file
    )
    .as_str(),
  );
  //EIN Channel Receiver der Info Messages aller Server
  let (info_tx, info_rx) = mpsc::channel();
  //Alle Channel Sender für Kommandos zu den SRCP Servern. Key ist die Busnummer.
  let mut all_cmd_tx: HashMap<usize, Sender<Message>> = HashMap::new();
  //Start aller über Konfiguration verlangter Modellbahn Schnittstellen Server
  //Alle belegten SRCP Busnummern, Key ist die Busnummer
  let mut aktive_srcp_busse: HashMap<usize, bool> = HashMap::new();
  for srcp_server in get_alle_srcp_server() {
    let mut srcpsrv = srcp_server.borrow_mut();

    if let Some(config_server_values) = config_file_values.get(srcpsrv.get_name()) {
      let bus_nr = config_server_values
        .get("bus")
        .ok_or(format!(
          "Keine bus-Angabe für Server {} vorhanden",
          srcpsrv.get_name()
        ))?
        .clone()
        .ok_or(format!(
          "Leere bus-Angabe für Server {} vorhanden",
          srcpsrv.get_name()
        ))?
        .parse::<usize>()
        .ok()
        .ok_or(format!(
          "Bus für Server {} nuss eine Zahl > 0 sein",
          srcpsrv.get_name()
        ))?;
      //Server wird verwendet, gültige Busnummer vorhanden
      for n in 0..srcpsrv.get_srcp_bus_count() {
        if aktive_srcp_busse.contains_key(&(bus_nr + n)) {
          error!(
            "SRCP bussnummer {} doppelt vergeben. Ignoriert für {}",
            bus_nr + n,
            srcpsrv.get_name()
          );
        } else {
          info!(
            "Neuer SRCP Server {} auf Bus {}",
            srcpsrv.get_name(),
            bus_nr + n
          );
          //Init nur auf erstem Bus (nur eine Instanz vorhanden)
          if n == 0 {
            if let Err(msg) = srcpsrv.init(bus_nr, &config_server_values) {
              error!("Error Server init: {}", msg);
              break;
            }
          }
          aktive_srcp_busse.insert(bus_nr + n, true);
        }
      }
      //Start Server wenn konfiguriert
      if aktive_srcp_busse.contains_key(&srcpsrv.get_busnr()) {
        let (cmd_tx, cmd_rx) = mpsc::channel();
        srcpsrv.start(cmd_rx, info_tx.clone());
        //Für alle SRCP Busse des Servers falls er mehrere unterstützt (wie z.B. S88)
        for sub_bus in 0..srcpsrv.get_srcp_bus_count() {
          all_cmd_tx.insert(srcpsrv.get_busnr() + sub_bus, cmd_tx.clone());
        }
      }
    }
  }
  //Sicherstellung Power Ausschalten und PID File gelöscht wird wenn Programm terminiert wird
  let all_cmd_tx_copy = all_cmd_tx.clone();
  thread::Builder::new()
    .name("Cleanup".to_string())
    .spawn(move || {
      terminate_poweroff(all_cmd_tx_copy);
      del_pidfile();
    })
    .unwrap();

  //Start srcp Server
  srcp::startup(&config_file_values, info_rx, &all_cmd_tx)
}

#[cfg(test)]
mod tests {
  use super::*;
  #[test]
  fn parse_cmd_line_test() {
    //Keine Kommandozeilenargumente
    let cmd_line_config = CmdLineConfig::parse_cmd_line(vec!["".to_string()].into_iter())
      .expect("Keine Kommandozeilen Argumente sind gültig");
    assert_eq!(cmd_line_config.fork, true);
    assert_eq!(
      cmd_line_config.config_file,
      format!("/etc/{}.conf", env!("CARGO_PKG_NAME"))
    );
    //-?
    let msg = CmdLineConfig::parse_cmd_line(vec!["".to_string(), "-?".to_string()].into_iter())
      .expect_err("Kommandozeilen Argument -? muss leeren Err liefern");
    assert_eq!(msg, "");
    //-blabla -> Error
    let msg =
      CmdLineConfig::parse_cmd_line(vec!["".to_string(), "-blabla".to_string()].into_iter())
        .expect_err("Unhültiges Kommandozeilen Argument muss Err liefern");
    assert_eq!(msg, "Unbekannter Parameter -blabla");
    //-f ohne File
    let msg = CmdLineConfig::parse_cmd_line(vec!["".to_string(), "-f".to_string()].into_iter())
      .expect_err("Keine Kommandozeilen Argumente sind gültig");
    assert_eq!(msg, "-f ohne Configfile");
    //-n und -f gültig
    let cmd_line_config = CmdLineConfig::parse_cmd_line(
      vec![
        "".to_string(),
        "-n".to_string(),
        "-f".to_string(),
        "configfilename".to_string(),
      ]
      .into_iter(),
    )
    .expect("Keine Kommandozeilen Argumente sind gültig");
    assert_eq!(cmd_line_config.fork, false);
    assert_eq!(cmd_line_config.config_file, "configfilename");
  }
}
