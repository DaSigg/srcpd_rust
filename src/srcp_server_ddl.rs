use std::{
  cell::RefCell,
  collections::HashMap,
  rc::Rc,
  sync::mpsc::{Receiver, Sender},
  thread,
  time::{Duration, Instant},
};

use gpio_cdev::{Chip, LineHandle, LineRequestFlags};
use log::{error, warn};
use spidev::{SpiModeFlags, Spidev, SpidevOptions};

use crate::{
  srcp_devices_ddl::{self},
  srcp_devices_ddl_gl::DdlGL,
  srcp_devices_ddl_sm::DdlSM,
  srcp_protocol_ddl::{HashMapProtokollVersion, HashMapVersion},
  srcp_protocol_ddl_dcc::{DccProtokoll, DccVersion},
  srcp_protocol_ddl_mfx::{MfxProtokoll, MfxVersion},
  srcp_protocol_ddl_mm::{MMProtokoll, MmVersion},
  srcp_server_types::{
    self, Message, SRCPMessage, SRCPMessageDevice, SRCPMessageID, SRCPMessageType, SRCPServer,
  },
};
use crate::{srcp_devices_ddl_ga::DdlGA, srcp_protocol_ddl_mm::SPI_BAUDRATE_MAERKLIN_LOCO_2};
use crate::{srcp_devices_ddl_power::DdlPower, srcp_protocol_ddl::DdlProtokolle};

/// Watchdog Timeout für Power Off
const WATCHDOG_TIMEOUT: Duration = Duration::from_secs(2);
/// Defaultpfad zum File für Speicherung Neuanmeldezähler
const PATH_REG_COUNTER_FILE: &str = "/etc/srcpd.regcount";
/// Thread Sleep wenn Power Off ist damit nicht 100% CPU Last vorhanden ist
const POWER_OFF_CPU_PAUSE: Duration = Duration::from_millis(10);
/// Input Prog Ack Signal GPIO 22 (= Pin 15, RI von RS232)
const GPIO_PROG_ACK: u32 = 22;

use lazy_static::lazy_static;
//Wegen V1 und 2 zwei Instanzen, beide brauchen ACK GPIO Input -> wird einmal hier erstellt.
lazy_static! {
  static ref GPIO_PROG_ACK_LINE_HANDLE: LineHandle = Chip::new("/dev/gpiochip0")
    .expect("/dev/gpiochip0 konnte nicht geöffnet werden")
    .get_line(GPIO_PROG_ACK)
    .expect("GPIO_MFX_RDS_QAL konnte nicht geöffnet werden")
    .request(LineRequestFlags::INPUT, 0, "input_dcc_prog_ack")
    .expect("GPIO_MFX_RDS_QAL konnte nicht als Input geöffnet werden");
}

pub struct DDL {
  //Konfiguration
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
  //UDP Basisportnummer für MFX RDS Rückmeldungen, wenn diese von GNU RADIO "mfxrds" kommen.
  //wenn nicht vorhanden: RDS Rückmeldungen von RDS Chip über GPIO
  udp_mfxrds_port: Option<u16>,
  //Pfad zu File zur Speicherung Neuanmeldezähler
  mfx_reg_count_file: String,
  //Booster mit On/Off mit "Siggmode" (Impuls auf RTS für On, Impuls auf DTR für Off)
  siggmode: bool,
  //DSR Booster GO Meldung Invers (bei nicht siggmode)
  dsr_invers: bool,
  //Verzögerung bis Abschaltung wegen Kurzschluss
  shortcut_delay: u64,
  //Wenn Siggmode: minimale Power On Zeit damit einmalig bei Ausschaltung
  //(wegen Kurzschluss) wieder versucht wird einzuschalten.
  timeout_shortcut_power_off: u64,
  //Watchdog aktiviert, automatische Power Ausschaltung wenn 2s lang keine Kommando empfangen wurde
  watchdog: bool,
  //Oszi Triggerkonfiguration aus Konfigfile
  trigger_port: Option<String>,
  trigger_gl: Option<String>,
  trigger_ga: Option<String>,
  trigger_sm: Option<String>,

  //Daten, werden nicht geklont
  //SPI Bus
  spidev: Option<Spidev>,
}
impl Clone for DDL {
  fn clone(&self) -> DDL {
    DDL {
      busnr: self.busnr,
      spiport: self.spiport.clone(),
      maerklin_enabled: self.maerklin_enabled,
      dcc_enabled: self.dcc_enabled,
      mfx_enabled_uid: self.mfx_enabled_uid,
      udp_mfxrds_port: self.udp_mfxrds_port,
      mfx_reg_count_file: self.mfx_reg_count_file.clone(),
      siggmode: self.siggmode,
      dsr_invers: self.dsr_invers,
      shortcut_delay: self.shortcut_delay,
      timeout_shortcut_power_off: self.timeout_shortcut_power_off,
      watchdog: self.watchdog,
      spidev: None, //Wird nie geklont
      trigger_port: self.trigger_port.clone(),
      trigger_gl: self.trigger_gl.clone(),
      trigger_ga: self.trigger_ga.clone(),
      trigger_sm: self.trigger_sm.clone(),
    }
  }
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
      udp_mfxrds_port: None,
      mfx_reg_count_file: PATH_REG_COUNTER_FILE.to_string(),
      siggmode: false,
      dsr_invers: false,
      shortcut_delay: 0,
      timeout_shortcut_power_off: 0,
      watchdog: false,
      spidev: None,
      trigger_port: None,
      trigger_gl: None,
      trigger_ga: None,
      trigger_sm: None,
    }
  }

  /// Liefert alle vorhandenen Protokollimplementierungen in allen Versionen zurück.
  /// Keys: Protokoll - Version
  /// Wenn zu einem Protokoll keine Versionsangabe vorhanden ist, dann wird 0 verwendet.
  fn get_all_protocols(&self) -> HashMapProtokollVersion {
    let mut all_protocols: HashMapProtokollVersion = HashMap::new();
    if self.maerklin_enabled {
      //MM
      let mut mm_protocols: HashMapVersion = HashMap::new();
      //MM V1
      mm_protocols.insert("1", Rc::new(RefCell::new(MMProtokoll::from(MmVersion::V1))));
      //MM V2
      mm_protocols.insert("2", Rc::new(RefCell::new(MMProtokoll::from(MmVersion::V2))));
      //MM V3
      mm_protocols.insert("3", Rc::new(RefCell::new(MMProtokoll::from(MmVersion::V3))));
      //MM V5
      mm_protocols.insert("5", Rc::new(RefCell::new(MMProtokoll::from(MmVersion::V5))));
      all_protocols.insert(DdlProtokolle::Maerklin, mm_protocols);
    }
    if self.dcc_enabled {
      //DCC
      let mut dcc_protocols: HashMapVersion = HashMap::new();
      //DCC V1
      dcc_protocols.insert(
        "1",
        Rc::new(RefCell::new(DccProtokoll::from(
          DccVersion::V1,
          &GPIO_PROG_ACK_LINE_HANDLE,
        ))),
      );
      //DCC V2
      dcc_protocols.insert(
        "2",
        Rc::new(RefCell::new(DccProtokoll::from(
          DccVersion::V2,
          &GPIO_PROG_ACK_LINE_HANDLE,
        ))),
      );
      all_protocols.insert(DdlProtokolle::Dcc, dcc_protocols);
    }
    if self.mfx_enabled_uid > 0 {
      //MFX
      let mut mfx_protocols: HashMapVersion = HashMap::new();
      //MFX V0
      mfx_protocols.insert(
        "0",
        Rc::new(RefCell::new(MfxProtokoll::from(
          MfxVersion::V0,
          self.mfx_enabled_uid,
          self.mfx_reg_count_file.clone(),
          self.udp_mfxrds_port,
        ))),
      );
      all_protocols.insert(DdlProtokolle::Mfx, mfx_protocols);
    }
    all_protocols
  }

  /// Liefert alle unterstützten Devices zurück
  /// # Arguments
  /// * tx - Channel Sender über den Info Messages zurück gesendet werden können
  fn get_all_devices(
    &self, tx: &Sender<SRCPMessage>,
  ) -> HashMap<
    srcp_server_types::SRCPMessageDevice,
    Rc<RefCell<dyn srcp_devices_ddl::SRCPDeviceDDL + '_>>,
  > {
    let all_protokolle = self.get_all_protocols();
    let mut all_devices: HashMap<
      SRCPMessageDevice,
      Rc<RefCell<dyn srcp_devices_ddl::SRCPDeviceDDL>>,
    > = HashMap::new();

    //Power Device
    all_devices.insert(
      SRCPMessageDevice::Power,
      Rc::new(RefCell::new(DdlPower::new(
        self.busnr,
        tx.clone(),
        self.siggmode,
        self.dsr_invers,
        self.shortcut_delay,
        self.timeout_shortcut_power_off,
      ))),
    );
    //GA Device
    all_devices.insert(
      SRCPMessageDevice::GA,
      Rc::new(RefCell::new(DdlGA::new(
        self.busnr,
        tx.clone(),
        &self.spidev,
        all_protokolle.clone(),
        self.trigger_port.clone(),
        self.trigger_ga.clone(),
      ))),
    );
    //GL Device
    all_devices.insert(
      SRCPMessageDevice::GL,
      Rc::new(RefCell::new(DdlGL::new(
        self.busnr,
        tx.clone(),
        &self.spidev,
        all_protokolle.clone(),
        self.trigger_port.clone(),
        self.trigger_gl.clone(),
      ))),
    );
    //SM Device
    all_devices.insert(
      SRCPMessageDevice::SM,
      Rc::new(RefCell::new(DdlSM::new(
        self.busnr,
        tx.clone(),
        all_protokolle.clone(),
        self.trigger_sm.clone(),
      ))),
    );
    all_devices
  }

  /// Ausführung als Thread
  /// # Arguments
  /// * rx - Channel Receiver über denn Kommandos empfangen werden
  /// * tx - Channel Sender über den Info Messages zurück gesendet werden können
  fn execute(&mut self, rx: Receiver<Message>, tx: Sender<SRCPMessage>) {
    //SPI Bus öffnen
    match Spidev::open(format!("{}.0", self.spiport)) {
      Ok(mut dev) => {
        let options = SpidevOptions::new()
          .bits_per_word(8)
          .max_speed_hz(SPI_BAUDRATE_MAERKLIN_LOCO_2) //Spielt hier keine Rolle, wird bei jedem Transfer individuell gesetzt
          .mode(SpiModeFlags::SPI_MODE_1)
          .build();
        if let Ok(()) = dev.configure(&options) {
          self.spidev = Some(dev);
        } else {
          error!(
            "DDL: SPI Device {} konnte nicht konfiguriert werden. Abbruch.",
            self.spiport
          );
          return;
        }
      }
      Err(msg) => {
        error!(
          "DDL: SPI Device {} konnte nicht geöffnet werden. Abbruch. {}",
          self.spiport, msg
        );
        return;
      }
    }
    //Warteschlange für alle SET ausser Power
    let mut queue: Vec<SRCPMessage> = Vec::new();
    //Zeitpunkt letztes empfangenes Kommando für Watchdog Überwachung
    let mut instant_kommando = Instant::now();

    //Alle unterstützten Devices
    let all_devices = self.get_all_devices(&tx);
    loop {
      //Power Device muss vorhanden sein, is_dev_spezifisch() liefert den Power Zustand
      let power_on = all_devices[&SRCPMessageDevice::Power]
        .borrow()
        .is_dev_spezifisch();
      //Immer alle ankommenden Kommandos auslesen
      loop {
        if let Ok(msg) = rx.try_recv() {
          match msg {
            Message::NewInfoClient { session_id } => {
              //Alle Devices müssen alle Zustände an neuen Info Client senden
              for (_key, device) in &all_devices {
                device.borrow().send_all_info(Some(session_id));
              }
            }
            Message::SRCPMessage { srcp_message } => {
              if let SRCPMessageID::Command { msg_type } = srcp_message.message_id {
                instant_kommando = Instant::now();
                match &all_devices.get(&srcp_message.device) {
                  //Nur Kommandomessages können (oder sollen) hier ankommen
                  Some(device) => {
                    if device.borrow().validate_cmd(&srcp_message) {
                      //SET Kommandos (ausser für Power Device und SM) kommen in die Warteschlange da sie
                      //1. nur bei Power On ausgegeben werden
                      //2. Lok Kommandos für die selbe Lok überholen sich, sprich wenn ein neues empfangen wurde ist
                      //   ein altes, noch nicht ausgegebenes, für diese Lok immer hinfällig
                      if (srcp_message.device == SRCPMessageDevice::Power)
                        || (srcp_message.device == SRCPMessageDevice::SM)
                        || (msg_type != SRCPMessageType::SET)
                      {
                        device
                          .try_borrow_mut()
                          .unwrap()
                          .execute_cmd(&srcp_message, power_on);
                      } else {
                        //Wenn es ein Lokkommando ist, dann ist altes Kommando für dieselbe Lok hinfällig
                        if srcp_message.device == SRCPMessageDevice::GL {
                          let adr = srcp_message.get_adr();
                          for i in 0..queue.len() {
                            let queue_msg = &queue[i];
                            if (queue_msg.device == SRCPMessageDevice::GL)
                              && (queue_msg.get_adr() == adr)
                            {
                              queue.remove(i);
                              //Wir können hier aufhören, es kann nur einen alten Eintrag gegeben haben
                              break;
                            }
                          }
                        }
                        //In Warteschlange
                        queue.push(srcp_message);
                      }
                    }
                  }
                  None => {
                    tx.send(SRCPMessage::new_err(
                      &srcp_message,
                      "421",
                      "unsupported device",
                    ))
                    .unwrap();
                  }
                }
              } else {
                warn!("DDL Empfang ignoriert: {}", srcp_message.to_string());
              }
            }
          }
        } else {
          break;
        }
      }
      //Wenn Power eingeschaltet ist, dann wird die Queue abgearbeitet
      if power_on {
        //Wenn Watchdog verlangt ist, dann machen wir hier noch dessen Kontrolle und Power off, wenn abgelaufen
        if self.watchdog && (Instant::now() > (instant_kommando + WATCHDOG_TIMEOUT)) {
          //Ausschaltkommando, Session ID 0 = srcp Server selbst
          all_devices[&SRCPMessageDevice::Power]
            .borrow_mut()
            .execute_cmd(
              &SRCPMessage::new(
                Some(0),
                self.busnr,
                SRCPMessageID::Command {
                  msg_type: (SRCPMessageType::SET),
                },
                SRCPMessageDevice::Power,
                vec!["OFF".to_string()],
              ),
              power_on,
            );
        } else {
          if queue.is_empty() {
            //Nicht zu tun -> Refresh für GL wenn vorhanden
            if let Some(dev) = all_devices.get(&SRCPMessageDevice::GL) {
              dev.try_borrow_mut().unwrap().send_refresh();
            }
          } else {
            //Alles was in Warteschlange ist, ist gültig, Device vorhanden und validiert
            //Erstes, ältestes Kommando ausführen
            let msg = queue.remove(0);
            all_devices
              .get(&msg.device)
              .unwrap()
              .try_borrow_mut()
              .unwrap()
              .execute_cmd(&msg, power_on);
          }
        }
      }
      //Allen Devices die Möglichkeit geben Hintergrundaufgaben abzuarbeiten, wenn vorhanden SM Antwort zurück senden
      let mut tel_gesendet = false;
      for (_, dev) in &all_devices {
        if dev.borrow_mut().execute(power_on) {
          tel_gesendet = true;
        }
      }
      if !power_on && !tel_gesendet {
        //Wenn Power On ist wird dauernd etwas gesendet. Die CPU "Pausen" kommen durch das SPI senden zu stande.
        //Wenn Power Off ist, wird (ausser SM verlangt etwas) nichts gesendet. Damit machen wir in diesem Loop 100% CPU Last für nichts.
        //Deshalb der CPU etwas Pausen gönnen, fallse bei Power Off wircklich nichts gesendet wurde.
        thread::sleep(POWER_OFF_CPU_PAUSE);
      }
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
    if let Some(mfx_reg_count_file) = config_file_bus.get("mfx_reg_count_file") {
      self.mfx_reg_count_file = mfx_reg_count_file
        .as_ref()
        .ok_or("DDL: zu mfx_reg_count_file muss ein Pfad angegegben werden.")?
        .clone();
    }
    if let Some(port) = config_file_bus.get("mfx_rds_port") {
      self.udp_mfxrds_port = Some(
        port
          .as_ref()
          .ok_or("DDL: MFX RDS Port > 0 (oder nichts) notwendig")?
          .parse::<u16>()
          .ok()
          .ok_or("MFX RDS Port muss eine Zahl > 0 sein")?,
      );
    }
    self.siggmode = config_file_bus.get("siggmode").is_some();
    self.dsr_invers = config_file_bus.get("dsr_invers").is_some();
    self.shortcut_delay = config_file_bus
      .get("shortcut_delay")
      .ok_or("DDL: shortcut_delay Parameter nicht vorhanden")?
      .as_ref()
      .ok_or("DDL: shortcut_delay Parameter ohne Wert")?
      .parse::<u64>()
      .ok()
      .ok_or("DDL: shortcut_delay Parameter muss eine Zahl >= 0 sein")?;
    if let Some(timeout_shortcut_power_off) = config_file_bus.get("timeout_shortcut_power_off") {
      self.timeout_shortcut_power_off = timeout_shortcut_power_off
        .as_ref()
        .ok_or("DDL: timeout_shortcut_power_off ohne Wert")?
        .parse::<u64>()
        .ok()
        .ok_or("DDL: timeout_shortcut_power_off muss eine Zahl >= 0 sein")?;
    }
    self.watchdog = config_file_bus.get("watchdog").is_some();
    if let Some(trigger_port) = config_file_bus.get("trigger_port") {
      self.trigger_port = trigger_port.clone();
    }
    if let Some(trigger_gl) = config_file_bus.get("trigger_gl") {
      self.trigger_gl = trigger_gl.clone();
    }
    if let Some(trigger_ga) = config_file_bus.get("trigger_ga") {
      self.trigger_ga = trigger_ga.clone();
    }
    if let Some(trigger_sm) = config_file_bus.get("trigger_sm") {
      self.trigger_sm = trigger_sm.clone();
    }
    Ok(())
  }

  /// Start dieses Servers
  /// # Arguments
  /// * rx - Channel Receiver über denn Kommandos empfangen werden
  /// * tx - Channel Sender über den Info Messages zurück gesendet werden können
  fn start(&self, rx: Receiver<Message>, tx: Sender<SRCPMessage>) {
    let mut instanz = self.clone();
    thread::Builder::new()
      .name("DDL_Thread".to_string())
      .spawn(move || instanz.execute(rx, tx))
      .unwrap();
  }
}
