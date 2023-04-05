use std::{
  collections::HashMap,
  sync::mpsc::{Receiver, Sender},
  thread,
  time::Duration,
};

use crate::srcp_server_types::{Message, SRCPServer};

#[derive(Clone)]
pub struct DDL {
  //SRCP Busnr
  busnr: usize,
  //SPI Port
  spiport: String,
  //Märklin Mototrola Protokoll aktiv
  maerklin_enabled: bool,
  //DCC Protokoll aktiv
  dcc_enabled: bool,
  //MFX Protokoll aktiv wenn UID > 0
  mfx_enabled_uid: u32,
  //Booster mit On/Off mit "Siggmode" (Impuls auf RTS für On, Impuls auf DTR für Off)
  siggmode: bool,
  //Verzögerung bis Abschaltung wegen Kurzschluss
  shortcut_delay: u32,
}

impl DDL {
  ///Neue Instanz erstellen
  pub fn new() -> DDL {
    DDL {
      busnr: 0,
      spiport: "".to_string(),
      maerklin_enabled: false,
      dcc_enabled: false,
      mfx_enabled_uid: 0,
      siggmode: false,
      shortcut_delay: 0,
    }
  }

  ///Ausführung als Thread
  /// # Arguments
  /// * rx - Channel Receiver über denn Kommandos empfangen werden
  /// * tx - Channel Sender über den Info Messages zurück gesendet werden können
  fn execute(&self, _rx: Receiver<Message>, _tx: Sender<Message>) -> ! {
    loop {
      //TODO
      thread::sleep(Duration::from_secs(1));
    }
  }
}

impl SRCPServer for DDL {
  /// Liefert den Name des SRCP Servers zurück
  /// Im Konfigfile muss für jeden verwendeten SRCP Server minimal ein Abschnitt mit diesem Name und dem zu verwenden Bus enthalten sein:
  /// [SRCPServerName]
  /// bus = x
  fn get_name(&self) -> &'static str {
    "ddl"
  }

  /// Liefert die Busnummer des SRCP Servers zurück, 0=nicht benutzt, konfiguriert
  fn get_busnr(&self) -> usize {
    self.busnr
  }

  /// Init dieses Servers
  /// Liefert Err zurück wenn ein Fehler aufgetreten ist (z.B. fehlender Konfig Parameter)
  /// # Arguments
  /// * busnr - Die SRCP Busnummers die diesem Server zugeordner ist.
  /// * config_file_bus - Der diesen Bus betreffende Teil des Konfigfiles
  fn init(
    &mut self, busnr: usize, config_file_bus: &HashMap<String, Option<String>>,
  ) -> Result<(), String> {
    self.busnr = busnr;
    self.spiport = config_file_bus
      .get("spiport")
      .ok_or("S88: spiport Parameter nicht vorhanden")?
      .clone()
      .ok_or("S88: spiport Parameter ohne Wert")?;
    self.maerklin_enabled = config_file_bus.get("maerklin").is_some();
    self.dcc_enabled = config_file_bus.get("dcc").is_some();
    if let Some(uid) = config_file_bus.get("mfx") {
      self.mfx_enabled_uid = uid
        .as_ref()
        .ok_or("DDL: MFX enable mit UID > 0 notwendig")?
        .parse::<u32>()
        .ok()
        .ok_or("MFX UID muss eine Zahl > 0 sein")?;
    }
    self.siggmode = config_file_bus.get("siggmode").is_some();
    self.shortcut_delay = config_file_bus
      .get("shortcut_delay")
      .ok_or("DDL: shortcut_delay Parameter nicht vorhanden")?
      .as_ref()
      .ok_or("DDL: shortcut_delay Parameter ohne Wert")?
      .parse::<u32>()
      .ok()
      .ok_or("DDL: shortcut_delay Parameter muss eine Zahl >= 0 sein")?;
    Ok(())
  }

  /// Start dieses Servers
  /// # Arguments
  /// * rx - Channel Receiver über denn Kommandos empfangen werden
  /// * tx - Channel Sender über den Info Messages zurück gesendet werden können
  fn start(&self, rx: Receiver<Message>, tx: Sender<Message>) {
    let instanz = self.clone();
    thread::spawn(move || instanz.execute(rx, tx));
  }
}
