//! globale Definitionen für alle SRCP-Server
use std::{
  collections::HashMap,
  sync::mpsc::{Receiver, Sender},
};

/// SRCP Message Angaben, Kommando odder Info (an einen oder alle)
#[derive(Clone, Debug)]
#[allow(dead_code)] //TODO
pub enum SRCPMessageDir {
  Info,                            //An alle SRCP Info Clients
  InfoSession { session_id: u32 }, //Info nur an Client mit dieser Session ID
  Command { session_id: u32 },
}

///SRCP Message
#[derive(Clone, Debug)]
#[allow(dead_code)] //TODO
pub enum SRCPMessageType {
  GET,
  SET,
  INIT,
  TERM,
}
#[derive(Clone, Debug)]
#[allow(dead_code)] //TODO
pub enum SRCPMessageDevice {
  //Generic Accessory
  GA,
  //Generic Loco
  GL,
  //Feddback
  FB { nr: usize, value: bool },
}

/// Eigentliche SRCP Message
#[derive(Clone, Debug)]
#[allow(dead_code)] //TODO
pub struct SRCPMessage {
  bus: usize,
  pub srcp_message_dir: SRCPMessageDir,
  srcp_type: SRCPMessageType,
  payload: SRCPMessageDevice,
}
impl SRCPMessage {
  pub fn new(
    bus: usize, srcp_message_dir: SRCPMessageDir, srcp_type: SRCPMessageType,
    payload: SRCPMessageDevice,
  ) -> SRCPMessage {
    SRCPMessage {
      bus,
      srcp_message_dir,
      srcp_type,
      payload,
    }
  }
}
impl ToString for SRCPMessage {
  fn to_string(&self) -> String {
    format!(
      "{}{} {} {}",
      match self.srcp_message_dir {
        SRCPMessageDir::Info => "INFO ",
        SRCPMessageDir::InfoSession { session_id: _ } => "INFO ",
        _ => "",
      },
      match self.srcp_type {
        SRCPMessageType::GET => "GET",
        SRCPMessageType::SET => "SET",
        SRCPMessageType::INIT => "INIT",
        SRCPMessageType::TERM => "TERM",
      },
      self.bus,
      match self.payload {
        SRCPMessageDevice::GA => "GA".to_string(),
        SRCPMessageDevice::GL => "GL".to_string(),
        SRCPMessageDevice::FB { nr, value } =>
          format!("FB {} {}", nr, if value { "1" } else { "0" }),
      },
    )
  }
}

/// Message Type für Kommunkation mit allen SRCP Servern
#[derive(Clone, Debug)]
#[allow(dead_code)] //TODO
pub enum Message {
  //Eigentliche SRCP Message
  SRCPMessage { srcp_message: SRCPMessage },
  //Information an SRCP Server dass ein neuer Info Client vorhanden ist -> allen aktuellen Zustände and diesen senden
  NewInfoClient { session_id: u32 },
}
#[allow(dead_code)] //TODO
impl Message {
  pub fn new_info_client(session_id: u32) -> Message {
    Message::NewInfoClient { session_id }
  }
  pub fn new_srcpmessage(srcp_message: SRCPMessage) -> Message {
    Message::SRCPMessage { srcp_message }
  }
}
impl ToString for Message {
  fn to_string(&self) -> String {
    match self {
      Message::SRCPMessage { srcp_message } => srcp_message.to_string(),
      Message::NewInfoClient { session_id } => format!("NewInfoClient session_id={}", session_id),
    }
  }
}

/// Schnittstelle, die alle SRCP Server implementieren müssen
pub trait SRCPServer {
  /// Liefert den Name des SRCP Servers zurück
  /// Im Konfigfile muss für jeden verwendeten SRCP Server minimal ein Abschnitt mit diesem Name und dem zu verwenden Bus enthalten sein:
  /// [SRCPServerName]
  /// bus = x
  fn get_name(&self) -> &'static str;
  /// Liefert die Busnummer des SRCP Servers zurück, 0=nicht benutzt, konfiguriert
  fn get_busnr(&self) -> usize;
  /// Liefert die Anzahl SRCP Busse, die durch diesen Server belegt werden
  fn get_srcp_bus_count(&self) -> usize {
    1
  }
  /// Init dieses Servers
  /// Liefert Err zurück wenn ein Fehler aufgetreten ist (z.B. fehlender Konfig Parameter)
  /// # Arguments
  /// * busnr - Die SRCP Busnummers die diesem Server zugeordner ist.
  /// * config_file_bus - Der diesen Bus betreffende Teil des Konfigfiles
  fn init(
    &mut self, busnr: usize, config_file_bus: &HashMap<String, Option<String>>,
  ) -> Result<(), String>;
  /// Start dieses Servers
  /// # Arguments
  /// * rx - Channel Receiver über denn Kommandos empfangen werden
  /// * tx - Channel Sender über den Info Messages zurück gesendet werden können
  fn start(&self, rx: Receiver<Message>, tx: Sender<Message>);
}
