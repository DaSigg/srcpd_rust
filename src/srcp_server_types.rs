//! globale Definitionen für alle SRCP-Server
use std::{
  collections::HashMap,
  sync::mpsc::{Receiver, Sender},
};

///SRCP Message
#[derive(Clone, Debug, PartialEq, Copy)]
pub enum SRCPMessageType {
  GET,
  SET,
  VERIFY,
  INIT,
  TERM,
}
impl ToString for SRCPMessageType {
  fn to_string(&self) -> String {
    match &self {
      SRCPMessageType::GET => "GET",
      SRCPMessageType::SET => "SET",
      SRCPMessageType::VERIFY => "VERIFY",
      SRCPMessageType::INIT => "INIT",
      SRCPMessageType::TERM => "TERM",
    }
    .to_string()
  }
}

/// SRCP Message Angaben, Kommando oder Info (an einen oder alle)
#[derive(Clone, Debug)]
pub enum SRCPMessageID {
  Info {
    //Info an einen oder alle SRCP Info Clients
    info_code: String,
  },
  Command {
    //Kommando von Session -> Info / Ok / Err Antwort muss an diese Session
    msg_type: SRCPMessageType,
  },
  Ok {
    //Ok an Client
    ok_code: String,
  },
  Err {
    //Error an Client
    err_code: String,
    err_text: String,
  },
}
impl ToString for SRCPMessageID {
  fn to_string(&self) -> String {
    match &self {
      SRCPMessageID::Info { info_code } => info_code.to_owned() + " INFO",
      SRCPMessageID::Command { msg_type } => msg_type.to_string(),
      SRCPMessageID::Ok { ok_code } => ok_code.to_owned() + " OK",
      SRCPMessageID::Err { err_code, err_text } => err_code.to_owned() + " ERROR " + err_text,
    }
  }
}
impl SRCPMessageID {
  /// Welche Message muss als String in Kurzform ohne Device und Parameter ausgegeben werden
  /// Return true für Kurzform
  pub fn str_kurz(&self) -> bool {
    match &self {
      SRCPMessageID::Info { info_code: _ } => false,
      SRCPMessageID::Command { msg_type: _ } => false,
      SRCPMessageID::Ok { ok_code: _ } => false,
      SRCPMessageID::Err {
        err_code: _,
        err_text: _,
      } => true,
    }
  }
}

#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub enum SRCPMessageDevice {
  //Generic Accessory
  GA,
  //Generic Loco
  GL,
  //Feedback
  FB,
  //Service Mode
  SM,
  //Power on/off
  Power,
}
impl ToString for SRCPMessageDevice {
  fn to_string(&self) -> String {
    match &self {
      SRCPMessageDevice::GA => "GA".to_string(),
      SRCPMessageDevice::GL => "GL".to_string(),
      SRCPMessageDevice::FB => "FB".to_string(),
      SRCPMessageDevice::SM => "SM".to_string(),
      SRCPMessageDevice::Power => "POWER".to_string(),
    }
  }
}

/// Eigentliche SRCP Message
#[derive(Clone, Debug)]
pub struct SRCPMessage {
  pub session_id: Option<u32>, //Von, an Client mit dieser Session ID, wenn bei Info nicht angegeben: an alle Info Clients
  pub bus: usize,
  pub message_id: SRCPMessageID,
  pub device: SRCPMessageDevice,
  pub parameter: Vec<String>,
}
impl SRCPMessage {
  /// Neue SRCPMessage erstellen
  pub fn new(
    session_id: Option<u32>, bus: usize, message_id: SRCPMessageID, device: SRCPMessageDevice,
    parameter: Vec<String>,
  ) -> SRCPMessage {
    SRCPMessage {
      session_id,
      bus,
      message_id,
      device,
      parameter,
    }
  }
  /// Neue SRCPMessage Ok erstellen
  /// # Arguments
  /// * msg - Kommandomessage aus der Session, Bus, Device kopiert werden.
  /// * ok_code - Zu verwendender OK Code
  pub fn new_ok(msg: &SRCPMessage, ok_code: &'static str) -> SRCPMessage {
    SRCPMessage {
      session_id: msg.session_id,
      bus: msg.bus,
      message_id: SRCPMessageID::Ok {
        ok_code: ok_code.to_string(),
      },
      device: msg.device.clone(),
      parameter: vec![],
    }
  }
  /// Neue SRCPMessage Error erstellen
  /// # Arguments
  /// * msg - Kommandomessage aus der Session, Bus, Device kopiert werden.
  /// * err_code - Zu verwendender OK Code
  /// * err_text - Zu verwendender Errortext
  pub fn new_err(msg: &SRCPMessage, err_code: &'static str, err_text: &'static str) -> SRCPMessage {
    SRCPMessage {
      session_id: msg.session_id,
      bus: msg.bus,
      message_id: SRCPMessageID::Err {
        err_code: err_code.to_string(),
        err_text: err_text.to_string(),
      },
      device: msg.device.clone(),
      parameter: vec![],
    }
  }
  /// Neue SRCPMessage Command aus String erstellen.
  /// Return Err, wenn Erstellungnicht möglich ist (zuwenig Parameter, unbekannte etc.)
  /// # Arguments
  /// * cmd - Commandline String Teile, getrennt an Spaces.
  ///         Es müssen min. 3 Teile SRCPMessageType BusNr und SRCPMessageDevice vorhanden sein.
  ///         Alle weiteren teile kommen, wenn vorhanden, in Parameter
  /// * session_id - Die Session, über die dieses Kommando empfangen wurde
  pub fn from(
    session_id: u32, cmd: &Vec<&str>,
  ) -> Result<SRCPMessage, (&'static str, &'static str)> {
    if cmd.len() < 3 {
      return Err(("419", "list too short"));
    }
    Ok(SRCPMessage {
      session_id: Some(session_id),
      message_id: SRCPMessageID::Command {
        msg_type: match cmd[0] {
          "GET" => SRCPMessageType::GET,
          "SET" => SRCPMessageType::SET,
          "VERIFY" => SRCPMessageType::VERIFY,
          "INIT" => SRCPMessageType::INIT,
          "TERM" => SRCPMessageType::TERM,
          &_ => return Err(("410", "unknown command")),
        },
      },
      bus: cmd[1].parse::<usize>().or(Err(("412", "wrong value")))?,
      device: match cmd[2] {
        "GA" => SRCPMessageDevice::GA,
        "GL" => SRCPMessageDevice::GL,
        "FB" => SRCPMessageDevice::FB,
        "SM" => SRCPMessageDevice::SM,
        "POWER" => SRCPMessageDevice::Power,
        &_ => return Err(("421", "unsupported device")),
      },
      parameter: cmd[3..].iter().map(|s| s.to_string()).collect(),
    })
  }
  /// Liefert die Adresse des Kommandos.
  /// Das ist, egal ob GA, GL immer der erste Parameter
  /// Return Err wenn keine Adresse vorhanden
  pub fn get_adr(&self) -> Option<u32> {
    if self.parameter.len() > 0 {
      return self.parameter[0].parse::<u32>().ok();
    }
    None
  }
}
impl ToString for SRCPMessage {
  fn to_string(&self) -> String {
    //INFO/SET/GET.... BusNr Device Parameter
    //OK
    //ERROR Text
    if self.message_id.str_kurz() {
      self.message_id.to_string()
    } else {
      format!(
        "{} {} {} {}",
        self.message_id.to_string(),
        self.bus,
        self.device.to_string(),
        {
          let mut p_str = String::from("");
          for p in &self.parameter {
            p_str += p.as_str();
            p_str += " ";
          }
          p_str
        }
      )
    }
  }
}

/// Message Type für Kommunkation mit allen SRCP Servern
#[derive(Clone, Debug)]
pub enum Message {
  //Eigentliche SRCP Message
  SRCPMessage { srcp_message: SRCPMessage },
  //Information an SRCP Server dass ein neuer Info Client vorhanden ist -> allen aktuellen Zustände and diesen senden
  NewInfoClient { session_id: u32 },
}
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
  fn start(&self, rx: Receiver<Message>, tx: Sender<SRCPMessage>);
}
