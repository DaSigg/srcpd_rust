use std::{
  sync::mpsc::Sender,
  time::{Duration, Instant},
};

use gpio::{
  sysfs::{SysFsGpioInput, SysFsGpioOutput},
  GpioIn, GpioOut, GpioValue,
};
use spidev::{Spidev, SpidevTransfer};

use crate::srcp_server_types::{SRCPMessage, SRCPMessageDevice, SRCPMessageID, SRCPMessageType};

//SPI Baudrate für Märklin / Motorola Protokoll.
//Diese wäre eigentlich genau 38461 Baud (1 Bit=26us, 1Byte=208us)
const SPI_BAUDRATE_MAERKLIN_LOCO: u32 = 38461;
//Nun macht der RaspberryPI aber leider nach jedem Byte Transfer auf dem SPI Bus 1 Bit Pause :-(
//Er macht diese 1 Bit / Clk Pause nicht mehr im DMA Mode. DMA wird ab Transfers
//von 96 Bytes verwendet (hoffe, das bleibt so ...).
//Um Märklin / Motorola Pakete auf 96 rauf zu bringen, wird folgendes gemacht:
//- die Baudrate Doppelt so gewählt wie notwendig (2 * 38461 für Loks)
//- die Wiederholung ins selbe Paket gepackt.
//- Pause am Anfang und vor Wiederholung mit 0 Bytes gefüllt.
pub const SPI_BAUDRATE_MAERKLIN_LOCO_2: u32 = 2 * SPI_BAUDRATE_MAERKLIN_LOCO;
const _SPI_BAUDRATE_MAERKLIN_FUNC_2: u32 = 2 * SPI_BAUDRATE_MAERKLIN_LOCO_2;

//SPI Baudrate für DCC/NMRA.
//Diese wird so gewählt, dass ein kurzer 58/58us Impuls (logisch 1) in einem Byte (0xF0) ausgegeben werden kann,
//ein langer 116/116us Impuls wird dann als 2 Bytes (0xFF, 0x00) ausgegeben.
//Damit hätten wird für 8 Bit also 116us -> 1 Bit 14.5us -> 68966 Baud.
//Damit werden NMRA/DCC Pakete inkl. der führenden Sync. Bytes leider nicht immer >= 96 Bytes -> DMA Mode und keine Pause nach 8 Bits
const _SPI_BAUDRATE_NMRA: u32 = 68966;
//Deshalb wird auch hier die doppelte Baudrate verwendet und dann wie folgt kodiert:
//1: 0xFF, 0x00
//0: 0xFF, 0xFF, 0x00, 0x00
const _SPI_BAUDRATE_NMRA_2: u32 = _SPI_BAUDRATE_NMRA * 2;

//SPI Baudrate für MFX.
//Auch hier müssen wir auf sicher 96 Bytes kommen um im DMA Modus zu sein und keine Pause zwischen den Bytes zu haben.
//1 Bit in MFX sind immer 100us. 1 Bit wird auf ein SPI Byte gelegt, also für ein Bit 100 / 8 = 12.5us -> 80000 Baud
//Grösse Paket wird damit (mit Sync Muster) für das kleinst mögliche Paket, das wir senden:
const _SPI_BAUDRATE_MFX: u32 = 80000;
//Minimales Paket das verwendet wird, ist "Fahren kurz" plus "Funktionen kurz"
// - 0            :  4 Bit -> Sicherstellen, dass Startpegel immer 0 ist
// - Sync         :  5 Bit
// - 10AAAAAAA    :  9 Bit Adresse (Minimal bei 7 Bit Adresse)
// - Kommando     :  7 Bit "Fahren kurz" 3+1+3
// - Kommando     :  7 Bit "Funktionen kurz" 3+4
// - Checksumme   :  8 Bit
// - Sync         : 10 Bit
// - Total        : 50 Bit
//Auch hier kommt nun deshalb wieder die doppelte Baudrate ins Spiel, damit wären wir bei 2*50=100 Bytes.
//1 MFX Bit -> 2 SPI Bytes
//Und mit mehr als 16 Funktionen haben wir ein neues minimales Paket:
// - 0            :  4 Bit -> Sicherstellen, dass Startpegel immer 0 ist
// - Sync         :  5 Bit
// - 10AAAAAAA    :  9 Bit Adresse (Minimal bei 7 Bit Adresse)
// - Kommando     : 12 Bit "Funktion einzeln" 3+7+1+1
// - Checksumme   :  8 Bit
// - Sync         : 10 Bit
// - Total        : 48 Bit
//Und Glück gehabt, wir sind mit mal 2 gerade auf 96 gekommen...
const _SPI_BAUDRATE_MFX_2: u32 = _SPI_BAUDRATE_MFX * 2;

/// Telegramm zum senden über SPI
pub struct DdlTel {
  //Die Baudrate mit der gesendet werden muss
  pub hz: u32,
  //Die Bytes die gesendet werden müssen
  pub daten: Vec<u8>,
}

/// Schnittstelle für alle Devices die in einem SRCP DDL Server bearbeitet werden
pub trait SRCPDeviceDDL {
  /// Empfangenes Kommando validieren
  /// Return true wenn Ok.
  /// Sendet die Antwort Message (Ok / Err) und wenn notwendig neue Zustände über Info Sender zurück.
  /// # Arguments
  /// * cmd_msg - Empfangenes Kommando
  fn validate_cmd(&self, cmd_msg: &SRCPMessage) -> bool;
  /// Empfangenes Kommando ausführen und versenden, ggf. interne Daten Updaten für späteren Refresh.
  /// Das Kommando muss gültig sein (validate_cmd), es wird hier nicht ,ehr überprüft.
  /// # Arguments
  /// * cmd_msg - Empfangenes Kommando
  fn execute_cmd(&mut self, cmd_msg: &SRCPMessage);
  /// Refresh Zyklus Telegramm senden (wird nur für GL aufgerufen)
  fn send_refresh(&self) {}
  /// Muss zyklisch aufgerufen werden. Erlaubt dem Device die Ausführung von
  /// von neuen Kommando oder refresh unabhängigen Aufgaben.
  fn execute(&mut self) {}
  /// Alle internen zustände als Info Message versenden
  /// # Arguments
  /// * session_id - SRCOP Client Session ID an die die Zustände gesendet werden sollen.
  ///                None -> Info an alle SRCP Clients
  fn send_all_info(&self, session_id: Option<u32>);
  /// Abfrage eines Device spezifischen Wertes / Zustandes
  fn is_dev_spezifisch(&self) -> bool {
    false
  }
  /// Senden von Schienentelegrammen über SPI Bus
  /// # Arguments
  /// * spidev - Geöffnetes SPI interface über das Telegramme zum Booster gesendet werden können
  /// * ddl_tel - Das zu sendende Telegramm
  fn send(&self, spidev: &Option<Spidev>, ddl_tel: &DdlTel) {
    let mut transfer = SpidevTransfer::write(ddl_tel.daten.as_slice());
    transfer.speed_hz = ddl_tel.hz;
    spidev
      .as_ref()
      .unwrap()
      .transfer(&mut transfer)
      .expect("DDL SPI write fail");
  }
}

/// Auf dem Raspberry PI ab V2 werden folgende Ports verwendet:
/// - CTS GPIO3 (=Pin5)
/// - RTS GPIO27 (=Pin13)
/// - DTR GPIO4 (=Pin7)
/// - DSR GPIO2 (=Pin3)
const CTS: u16 = 3;
const RTS: u16 = 27;
const DTR: u16 = 4;
const DSR: u16 = 2;
/// Dauer Start- Stop Impuls siggmode
const DAUER_STOP_IMPULS_SIGG_MODE: Duration = Duration::from_millis(500);
const DAUER_START_IMPULS_SIGG_MODE: Duration = Duration::from_millis(750);
/// Leitungen zum Booster ON ist 0 wegen Invertierung durch RS232 Treiber 0V->12V / 3.3V->-12V
const RS232_ON: GpioValue = GpioValue::Low;
const RS232_OFF: GpioValue = GpioValue::High;
/// Device Power für DDL
/// Power On Off:
/// - siggmode: Booster GO message on CTS Line, Booster GO / STOP Command impluse on RTS/DTR
/// - sonst: Booster GO message on CTS Line mit shortcut_delay, Booster GO / STOP Command DTR (dauerhaft)
pub struct DdlPower {
  //SRCP Bus auf dem gearbeitet wird
  bus: usize,
  //Sender für SRCP Antworten
  tx: Sender<SRCPMessage>,
  //Konfiguration Power On/Off über Impulse
  siggmode: bool,
  //DSR Booster GO Meldung Invers (bei nicht siggmode)
  dsr_invers: bool,
  //Verzögerungszeit in ms bis Ausschaltung bei Überlast wenn NICHT siggmode
  shortcut_delay: Duration,
  //Aktueller Power Zustand
  power_on: bool,
  //Zeitpunkt Start/Stopimpulse wieder ausschalten siggmode
  impuls_aus: Instant,
  //Letzter Zeitpunkt Booster OK (kein Kurzsschluss) bei nicht siggmode
  kein_shortcut: Instant,
  //Booster Go Meldung siggmode
  gpio_cts_go_in: SysFsGpioInput,
  //Booster Go Meldung / Shortcut
  gpio_dsr_go_in: SysFsGpioInput,
  //Booster Go Ausgang (Impuls bei siggmode)
  gpio_rts_go_out: SysFsGpioOutput,
  //Booster Stop Ausgang Impuls bei siggmode
  gpio_dtr_stop_out: SysFsGpioOutput,
}
impl DdlPower {
  /// Neue Instanz erstellen
  /// # Arguments
  /// * bus - SRCP Bus auf dem dieses Device arbeitet
  /// * tx - Sender für Info Messages / Antworten an SRCP Clients
  /// * siggmode - Impulse für Booster Start/Stop
  pub fn new(
    bus: usize, tx: Sender<SRCPMessage>, siggmode: bool, dsr_invers: bool, shortcut_delay: u64,
  ) -> DdlPower {
    let mut result = DdlPower {
      bus: bus,
      tx: tx,
      siggmode: siggmode,
      dsr_invers: dsr_invers,
      shortcut_delay: Duration::from_millis(shortcut_delay),
      power_on: false,
      impuls_aus: Instant::now(),
      kein_shortcut: Instant::now(),
      gpio_cts_go_in: gpio::sysfs::SysFsGpioInput::open(CTS)
        .expect(format!("GPIO {} konnte nicht geöffnet werden", CTS).as_str()),
      gpio_dsr_go_in: gpio::sysfs::SysFsGpioInput::open(DSR)
        .expect(format!("GPIO {} konnte nicht geöffnet werden", DSR).as_str()),
      gpio_rts_go_out: gpio::sysfs::SysFsGpioOutput::open(RTS)
        .expect(format!("GPIO {} konnte nicht geöffnet werden", RTS).as_str()),
      gpio_dtr_stop_out: gpio::sysfs::SysFsGpioOutput::open(DTR)
        .expect(format!("GPIO {} konnte nicht geöffnet werden", DTR).as_str()),
    };
    //Default setzen, Ausgänge ausgeschaltet
    result.gpio_rts_go_out.set_value(RS232_OFF).unwrap();
    result.gpio_dtr_stop_out.set_value(RS232_OFF).unwrap();
    result
  }
  /// Neuer Power Zustand übernehmen
  /// # Arguments
  /// * power - Neuer Power Zustand
  fn set_power(&mut self, power: bool) {
    if self.power_on != power {
      self.power_on = power;
      self.send_all_info(None);
      if self.siggmode {
        if power {
          //Startimpuls
          self.gpio_rts_go_out.set_value(RS232_ON).unwrap();
          self.gpio_dtr_stop_out.set_value(RS232_OFF).unwrap();
          self.impuls_aus = Instant::now() + DAUER_START_IMPULS_SIGG_MODE;
        } else {
          //Stopimpuls
          self.gpio_dtr_stop_out.set_value(RS232_ON).unwrap();
          self.gpio_rts_go_out.set_value(RS232_OFF).unwrap();
          self.impuls_aus = Instant::now() + DAUER_STOP_IMPULS_SIGG_MODE;
        }
      } else {
        //Booster On mit Dauerausgabe an RTS
        self
          .gpio_rts_go_out
          .set_value(if power { RS232_ON } else { RS232_OFF })
          .unwrap();
      }
    }
  }
}
impl SRCPDeviceDDL for DdlPower {
  /// Empfangenes Kommando validieren
  /// Return true wenn Ok.
  /// Sendet die Antwort Message (Ok / Err) und wenn notwendig neue Zustände über Info Sender zurück.
  /// # Arguments
  /// * cmd_msg - Empfangenes Kommando
  fn validate_cmd(&self, cmd_msg: &SRCPMessage) -> bool {
    //SET/GET <bus> POWER [ON|OFF] [freetext]
    //Hier muss nur noch SET|GET & ON|OFF kontrolliert werden
    let mut cmd_get = false;
    if match &cmd_msg.message_id {
      SRCPMessageID::Command { msg_type } => {
        cmd_get = *msg_type == SRCPMessageType::GET;
        cmd_get || (*msg_type == SRCPMessageType::SET)
      }
      _ => false,
    } && (match cmd_msg.parameter.get(0) {
      Some(para) => (para == "ON") || (para == "OFF"),
      None => false,
    } || cmd_get)
    {
      if cmd_get {
        self.send_all_info(cmd_msg.session_id);
      } else {
        self.tx.send(SRCPMessage::new_ok(cmd_msg, "200")).unwrap();
      }
      true
    } else {
      self
        .tx
        .send(SRCPMessage::new_err(cmd_msg, "412", "wrong value"))
        .unwrap();
      false
    }
  }

  /// Empfangenes Kommando ausführen und versenden, ggf. interne Daten Updaten für späteren Refresh.
  /// Das Kommando muss gültig sein (validate_cmd), es wird hier nicht mehr überprüft.
  /// # Arguments
  /// * cmd_msg - Empfangenes Kommando
  fn execute_cmd(&mut self, cmd_msg: &SRCPMessage) {
    //Nur das SET Kommando muss hier ausgeführt werden
    match &cmd_msg.message_id {
      SRCPMessageID::Command { msg_type } => {
        if *msg_type == SRCPMessageType::SET {
          self.set_power(cmd_msg.parameter[0] == "ON");
        }
      }
      _ => {}
    }
  }

  /// Alle internen zustände als Info Message versenden
  /// # Arguments
  /// * session_id - SRCP Client Session ID an die die Zustände gesendet werden sollen.
  ///                None -> Info an alle SRCP Clients
  fn send_all_info(&self, session_id: Option<u32>) {
    //Hier gibt es nur den aktuellen Power Zustand
    self
      .tx
      .send(SRCPMessage::new(
        session_id,
        self.bus,
        SRCPMessageID::Info {
          info_code: "100".to_string(),
        },
        SRCPMessageDevice::Power,
        vec![if self.power_on {
          "ON".to_string()
        } else {
          "OFF".to_string()
        }],
      ))
      .unwrap();
  }
  /// Abfrage eines Device spezifischen Wertes / Zustandes
  /// Liefert hier den Power Zustand
  fn is_dev_spezifisch(&self) -> bool {
    self.power_on
  }

  /// Hintergrundaktivität:
  /// - Ausschalten Start- Stopimpulse zu Booster wenn siggmode
  /// - Kontrolle Boosterrückmeldung On/Off (Shortcut)
  fn execute(&mut self) {
    if self.siggmode {
      //Start- Stop Impulse aus
      if Instant::now() > self.impuls_aus {
        self.gpio_rts_go_out.set_value(RS232_OFF).unwrap();
        self.gpio_dtr_stop_out.set_value(RS232_OFF).unwrap();
        //Booster aus Erkennung nach Impulsausgabe
        let booster_on = self.gpio_cts_go_in.read_value().unwrap() == RS232_ON;
        //Aus- und Einschalten vom Booster übernehmen
        self.set_power(booster_on);
      }
    } else {
      //Kurzschluss- Erkennung
      let booster_on = (self.gpio_dsr_go_in.read_value().unwrap() == RS232_ON) ^ self.dsr_invers;
      //Wenn Booster ein und Rückmeldung ein -> jetzt ist kein Kurzsschluss
      //Aber auch, damit überhaupt eingeschaltet werden kann, wenn Booster aus ist -> kein Kurzschluss
      if booster_on || (!self.power_on) {
        self.kein_shortcut = Instant::now();
      } else {
        //Booster sollte ein sein, Rückmeldung ist aber aus -> nach Timeout ganz ausschalten
        if Instant::now() > (self.kein_shortcut + self.shortcut_delay) {
          self.set_power(false);
        }
      }
    }
  }
}
