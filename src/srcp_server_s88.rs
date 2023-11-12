use std::{
  collections::HashMap,
  io::Read,
  sync::mpsc::{Receiver, Sender},
  thread,
  time::Duration,
};

use crate::srcp_server_types::{
  Message, SRCPMessage, SRCPMessageDevice, SRCPMessageID, SRCPMessageType, SRCPServer,
};
use gpio::{sysfs::SysFsGpioOutput, GpioOut, GpioValue};
use log::warn;
use spidev::{SpiModeFlags, Spidev, SpidevOptions};

/// Max. Anzahl unterstützer S88 Busse (= Anzahl belegter SRCP Busse)
const MAX_S88: usize = 4;
/// Frequenz SPI Bus für S88
/// Leider ist bei allen SPI interfaces die kleinst mögliche Taktrate core_freq / (2 * (speed + 1)).
/// speed ist je 12 Bit, max. also 4095.
/// Bei 250MHz sind wir also bei 30.5kHz. Das geht bei mir noch stabil, 50 kHz geht nicht mehr stabil.
/// -> core_freq darf nicht grösser als 250MHz gesetzt werden!
/// Die 20 kHz sind ein Wunsch, der auf SPI des Broadcom Chips nicht in Erfüllung geht .... :-(
const SPI_HZ: u32 = 20_000;
/// maximal number of bytes read from one s88-bus
const S88_MAXPORTSB: usize = 64;
/// Pause zwischen 2 SPI Transfers damit alle CE Leitungen sicher minimale Zeit auf 1 sind
const PAUSE_SPI_TRANSFER: Duration = Duration::from_micros(500);

#[derive(Clone)]
pub struct S88 {
  //SRCP Busnr
  busnr: usize,
  //Refreshzeit in ms
  refresh: u64,
  //Anzahl Wiederholungen für Filterung, sollte ungerade sein.
  repeat: usize,
  //SPI Port
  spiport: String,
  //SPI Mode
  spimode: u32,
  //Anzahl einzulesende Bytes Bus1..4
  number_bytes: [usize; MAX_S88],
  //Konfiguration Oszi Trigger pro S88 Bus und Feedbacknummer
  trigger_port: Option<u16>,
  trigger: [Vec<usize>; MAX_S88],
}

impl S88 {
  ///Neue Instanz erstellen
  pub fn new() -> S88 {
    S88 {
      busnr: 0,
      refresh: 50,
      repeat: 3,
      spiport: "".to_string(),
      spimode: 1,
      //Für alle 4 S88 Busse
      number_bytes: [0; MAX_S88],
      trigger_port: None,
      trigger: [vec![], vec![], vec![], vec![]],
    }
  }

  ///Ausführung als Thread
  /// # Arguments
  /// * rx - Channel Receiver über denn Kommandos empfangen werden
  /// * tx - Channel Sender über den Info Messages zurück gesendet werden können
  fn execute(&self, rx: Receiver<Message>, tx: Sender<SRCPMessage>) {
    let mut spidevs: Vec<Option<Spidev>> = Vec::new();
    //SPI Interfaces für alle Konfigurierten S88 Busse (number_bytes>0) öffnen
    for (i, number) in self.number_bytes.iter().enumerate() {
      spidevs.push(Option::None);
      if *number > 0 {
        match Spidev::open(format!("{}.{}", self.spiport, i)) {
          Ok(mut dev) => {
            let options = SpidevOptions::new()
              .bits_per_word(8)
              .max_speed_hz(SPI_HZ)
              .mode(SpiModeFlags::from_bits_truncate(self.spimode))
              .build();
            if let Ok(()) = dev.configure(&options) {
              spidevs[i] = Some(dev);
            } else {
              warn!(
                "S88: SPI Device {}.{} konnte nicht konfiguriert werden.",
                self.spiport, i
              );
            }
          }
          Err(msg) => {
            warn!(
              "S88: SPI Device {}.{} konnte nicht geöffnet werden: {}",
              self.spiport, i, msg
            );
          }
        }
      }
    }

    let mut akt_wiederhol_index: usize = 0;
    //SPI Buffer [SPIBus][Wiederholung][Byte]
    let mut s88_input_buffer: Vec<Vec<Vec<u8>>> = vec![vec![vec![]; self.repeat]; MAX_S88];
    //Für jeden Bus die aktuellen und letzten S88 Zustände
    let mut s88_states: Vec<Vec<bool>> = vec![vec![]; MAX_S88];
    //Anzahl Byte pro Bus gemäss Konfiguration setzen
    for spi_bus in 0..MAX_S88 {
      s88_states[spi_bus].resize(self.number_bytes[spi_bus] * 8, false);
      for repeat in 0..self.repeat {
        s88_input_buffer[spi_bus][repeat].resize(self.number_bytes[spi_bus], 0);
        s88_input_buffer[spi_bus][repeat].shrink_to_fit();
      }
    }
    //Damit nur einmal gerechnet werden muss
    let filter_grenzwert = self.repeat / 2;
    //Wenn Oszi Trigger konfiguriert sind: IO Port öffnen
    let mut trigger_port: Option<SysFsGpioOutput> = None;
    if let Some(port) = self.trigger_port {
      for trigger in &self.trigger {
        if !trigger.is_empty() {
          trigger_port = Some(
            SysFsGpioOutput::open(port)
              .expect(format!("GPIO für Oszi Trigger konnte nicht geöffnet werden").as_str()),
          );
          break;
        }
      }
    }
    //Und ab an die Arbeit, einlesen, auswerten, Veränderungen melden, warten und wieder von vorn ...
    loop {
      //Wenn ein Triggerport konfiguriert ist: zu Beginn mal auf 0 setzen.
      if trigger_port.is_some() {
        trigger_port
          .as_mut()
          .unwrap()
          .set_value(GpioValue::Low)
          .unwrap();
      }
      //SPI Einlesen
      for spi_bus in 0..MAX_S88 {
        if spidevs[spi_bus].is_some() {
          //Bus geöffnet vorhanden
          spidevs[spi_bus]
            .as_mut()
            .unwrap()
            .read(s88_input_buffer[spi_bus][akt_wiederhol_index].as_mut_slice())
            .expect("S88 SPI read fail");
          //Damit sicher alle CE Leitungen gemeinsam eine minimale Zeit auf 1 zurück sind zwischen den beiden Transfers etwas warten
          thread::sleep(PAUSE_SPI_TRANSFER);
        }
      }
      //Mehrheitsentscheid über alle verlangten Wiederholungen
      //Damit nicht jedes mal geschoben werden muss, Bit Order wie von S88 -> LSB kommt zuerst
      const BIT_VALUES: [u8; 8] = [
        1 << 7,
        1 << 6,
        1 << 5,
        1 << 4,
        1 << 3,
        1 << 2,
        1 << 1,
        1 << 0,
      ];

      //Über alle S88 Busse
      for spi_bus in 0..MAX_S88 {
        //Über alle Bytes des Busses
        for byte_nr in 0..self.number_bytes[spi_bus] {
          //Über alle Bits im Byte
          for bit_nr in 0..8 {
            //Und noch die Wiederholungen
            let mut count: usize = 0;
            for w in 0..self.repeat {
              if (s88_input_buffer[spi_bus][w][byte_nr] & BIT_VALUES[bit_nr]) != 0 {
                count += 1;
              }
            }
            let state = count > filter_grenzwert;
            let fb_nr = byte_nr * 8 + bit_nr;
            if state != s88_states[spi_bus][fb_nr] {
              //Veränderung, senden
              s88_states[spi_bus][fb_nr] = state;
              let msg = SRCPMessage::new(
                None,
                self.busnr + spi_bus, //die S88 Busse gehen auf unterschiedliche SRCP Busnummern
                SRCPMessageID::Info {
                  info_code: "100".to_string(),
                },
                SRCPMessageDevice::FB,
                vec![(fb_nr + 1).to_string(), (state as usize).to_string()], //Nummerierung bei SRCP beginnt bei 1
              );
              match tx.send(msg) {
                Err(msg) => {
                  warn!("S88 execute send Error, wird beendet: {}", msg);
                  break;
                }
                Ok(_) => {}
              }
            }
            //Wenn ein Trigger für diesen FB konfiguriert ist: bei jeder Veränderung (ohne Filter) gegenüber gespeichertem (gefiltertem) Wert senden.
            if trigger_port.is_some()
              && self.trigger[spi_bus].contains(&fb_nr)
              && (s88_states[spi_bus][fb_nr]
                != ((s88_input_buffer[spi_bus][akt_wiederhol_index][byte_nr] & BIT_VALUES[bit_nr])
                  != 0))
            {
              trigger_port
                .as_mut()
                .unwrap()
                .set_value(GpioValue::High)
                .unwrap();
            }
          }
        }
      }

      //Prüfen ob neuer Info Client alle Daten haben muss
      match rx.try_recv() {
        Ok(msg) => {
          match msg {
            Message::NewInfoClient { session_id } => {
              //Neuer Info Client, alle Zustände senden, alle FB die true sind
              for spi_bus in 0..MAX_S88 {
                for fb_nr in 0..s88_states[spi_bus].len() {
                  let state = s88_states[spi_bus][fb_nr];
                  if state {
                    let msg = SRCPMessage::new(
                      Some(session_id),
                      self.busnr + spi_bus, //die S88 Busse gehen auf unterschiedliche SRCP Busnummern
                      SRCPMessageID::Info {
                        info_code: "100".to_string(),
                      },
                      SRCPMessageDevice::FB,
                      vec![(fb_nr + 1).to_string(), (state as usize).to_string()], //Nummerierung bei SRCP beginnt bei 1
                    );
                    if let Err(msg) = tx.send(msg) {
                      warn!("S88 execute send Error, wird beendet: {}", msg);
                      break;
                    }
                  }
                }
              }
            }
            Message::SRCPMessage { srcp_message } => {
              let mut send_error = true;
              //Alles andere als GET FB ist hier nicht relevant, S88 kann keine anderen Kommandos ausführen -> Error
              match srcp_message.message_id {
                SRCPMessageID::Command { msg_type } => {
                  if (msg_type == SRCPMessageType::GET)
                    && (srcp_message.device == SRCPMessageDevice::FB)
                    && (srcp_message.parameter.len() > 0)
                  {
                    if let Ok(fb_nr) = srcp_message.parameter[0].parse::<usize>() {
                      //SRCP Nummern beginnen bei 1
                      if (fb_nr > 0) && (s88_states[srcp_message.bus - self.busnr].len() >= fb_nr) {
                        send_error = false;
                        if let Err(msg) = tx.send(SRCPMessage {
                          session_id: Some(srcp_message.session_id.unwrap()),
                          bus: srcp_message.bus,
                          message_id: SRCPMessageID::Info {
                            info_code: "100".to_string(),
                          },
                          device: SRCPMessageDevice::FB,
                          parameter: vec![
                            if s88_states[srcp_message.bus - self.busnr][fb_nr - 1] {
                              "1".to_string()
                            } else {
                              "0".to_string()
                            },
                          ],
                        }) {
                          warn!("S88 execute send Error, wird beendet: {}", msg);
                          break;
                        }
                      }
                    }
                  }
                }
                _ => {}
              }
              if send_error {
                if let Err(msg) = tx.send(SRCPMessage {
                  session_id: Some(srcp_message.session_id.unwrap()),
                  bus: srcp_message.bus,
                  message_id: SRCPMessageID::Err {
                    err_code: "420".to_string(),
                    err_text: "unsupported device protocol".to_string(),
                  },
                  device: SRCPMessageDevice::FB,
                  parameter: vec![],
                }) {
                  warn!("S88 execute send Error, wird beendet: {}", msg);
                  break;
                }
              }
            }
          }
        }
        Err(_) => {} //Nichts empfangen
      }
      //Nächster Filterplatz
      akt_wiederhol_index += 1;
      if akt_wiederhol_index >= self.repeat {
        akt_wiederhol_index = 0;
      }
      thread::sleep(Duration::from_millis(self.refresh));
    }
  }
}

impl SRCPServer for S88 {
  /// Liefert den Name des SRCP Servers zurück
  /// Im Konfigfile muss für jeden verwendeten SRCP Server minimal ein Abschnitt mit diesem Name und dem zu verwenden Bus enthalten sein:
  /// [SRCPServerName]
  /// bus = x
  fn get_name(&self) -> &'static str {
    "s88"
  }

  /// Liefert die Busnummer des SRCP Servers zurück, 0=nicht benutzt, konfiguriert
  fn get_busnr(&self) -> usize {
    self.busnr
  }

  /// Liefert die Anzahl SRCP Busse, die durch diesen Server belegt werden
  /// S88: es werden 4 S88 Busse unterstützt.
  fn get_srcp_bus_count(&self) -> usize {
    MAX_S88
  }

  /// Init dieses Servers
  /// Liefert Err zurück wenn ein Fehler aufgetreten ist (z.B. fehlender Konfig Parameter)
  /// # Arguments
  /// * busnr - Die SRCP Busnummers die diesem Server zugeordner ist.
  /// * config_file_bus - Der diesen Bus betreffende Teil des Konfigfiles
  /// S88 Bus hat folgende Konfigparameter:
  /// refresh Refreshzeit in ms
  /// spiport SPI Portname
  /// spimode SPI Mode (1 wenn möglich, 2 mit Zusatzschaltung)
  /// number_fb_1 Anzahl S88 Module (=16 Bit) an 1. S88 Bus
  /// number_fb_2 Anzahl S88 Module (=16 Bit) an 2. S88 Bus
  /// number_fb_3 Anzahl S88 Module (=16 Bit) an 3. S88 Bus
  /// number_fb_4 Anzahl S88 Module (=16 Bit) an 4. S88 Bus
  /// Optional:
  /// trigger_fb_1, trigger_fb_2, trigger_fb_3, trigger_fb_4
  /// mit Liste der FB's bei deren veränderung ein Oszi Triggerimpuls ausgegeben werden soll.
  fn init(
    &mut self, busnr: usize, config_file_bus: &HashMap<String, Option<String>>,
  ) -> Result<(), String> {
    self.busnr = busnr;
    self.refresh = config_file_bus
      .get("refresh")
      .ok_or("S88: refresh Parameter nicht vorhanden")?
      .clone()
      .ok_or("S88: refresh Parameter ohne Wert")?
      .parse::<u64>()
      .ok()
      .ok_or("S88 refresh muss eine Zahl sein")?;
    self.repeat = config_file_bus
      .get("repeat")
      .ok_or("S88: repeat Parameter nicht vorhanden")?
      .clone()
      .ok_or("S88: repeat Parameter ohne Wert")?
      .parse::<usize>()
      .ok()
      .ok_or("S88 repeat muss eine Zahl sein")?;
    self.spiport = config_file_bus
      .get("spiport")
      .ok_or("S88: spiport Parameter nicht vorhanden")?
      .clone()
      .ok_or("S88: spiport Parameter ohne Wert")?;
    self.spimode = config_file_bus
      .get("spimode")
      .ok_or("S88: spimode Parameter nicht vorhanden")?
      .clone()
      .ok_or("S88: spimode Parameter ohne Wert")?
      .parse::<u32>()
      .ok()
      .ok_or("S88 spimode muss 1 oder 2 sein")?;
    if (self.spimode != SpiModeFlags::SPI_MODE_1.bits())
      && (self.spimode != SpiModeFlags::SPI_MODE_2.bits())
    {
      Err("S88 spimode muss 1 oder 2 sein")?;
    }
    for i in 0..self.number_bytes.len() {
      //Anzahl S88 Module pro S88 Bus
      let name = format!("number_fb_{}", i + 1);
      self.number_bytes[i] = config_file_bus
        .get(&name)
        .ok_or(format!("S88: {} Parameter nicht vorhanden", name))?
        .clone()
        .ok_or(format!("S88: {} Parameter ohne Wert", name))?
        .parse::<usize>()
        .ok()
        .ok_or(format!("S88 {} muss eine Zahl sein", name))?
        * 2; //16 Bit pro S88 Modul
      if self.number_bytes[i] > S88_MAXPORTSB {
        warn!(
          "S88: Max. {} pro Bus wird unterstützt. Konfiguriert für Bus {} sind {}.",
          S88_MAXPORTSB,
          i + 1,
          self.number_bytes[i]
        );
        self.number_bytes[i] = S88_MAXPORTSB;
      }
      //Optionale Oszi Trigger pro S88 Bus
      if let Some(trigger_port_option) = config_file_bus.get("trigger_port") {
        if let Some(trigger_port_port) = trigger_port_option {
          if let Ok(trigger_port_port_nr) = trigger_port_port.parse::<u16>() {
            self.trigger_port = Some(trigger_port_port_nr);
            //Wenn ein Triggerport definiert ist, dann macht es Sinn, die restliche Trigger Konfiguration zu verarbeiten
            let name = format!("trigger_fb_{}", i + 1);
            if let Some(trigger_fb_option) = config_file_bus.get(&name) {
              if let Some(trigger_fb) = trigger_fb_option {
                for trigger in trigger_fb.split(",") {
                  if let Ok(fb_nr) = trigger.parse::<usize>() {
                    //Auf SRCP beginnen die FB Nummern bei 1
                    if (fb_nr > 0) && (fb_nr <= self.number_bytes[i] * 16) {
                      self.trigger[i].push(fb_nr - 1);
                    } else {
                      warn!(
                        "S88 Bus {}: Ungültige Trigger Konfiguration FB Nummer: {}. Erlaubt 1 bis {}.",
                        i + 1,
                        trigger,
                        self.number_bytes[i] * 16
                      );
                    }
                  } else {
                    warn!(
                      "S88 Bus {}: Ungültige Trigger Konfiguration: {}.",
                      i + 1,
                      trigger
                    );
                  }
                }
              }
            }
          } else {
            warn!("S88 Bus: Triggerport muss eine poistive Zahl sein.");
          }
        }
      }
    }
    Ok(())
  }

  /// Start dieses Servers
  /// # Arguments
  /// * rx - Channel Receiver über denn Kommandos empfangen werden
  /// * tx - Channel Sender über den Info Messages zurück gesendet werden können
  fn start(&self, rx: Receiver<Message>, tx: Sender<SRCPMessage>) {
    let instanz = self.clone();
    thread::Builder::new()
      .name("S88_Thread".to_string())
      .spawn(move || instanz.execute(rx, tx))
      .unwrap();
  }
}
