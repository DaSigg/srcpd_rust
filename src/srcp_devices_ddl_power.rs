use std::{
  sync::mpsc::Sender,
  time::{Duration, Instant},
};

use gpio_cdev::{Chip, LineHandle, LineRequestFlags};

use crate::{
  srcp_devices_ddl::SRCPDeviceDDL,
  srcp_server_types::{SRCPMessage, SRCPMessageDevice, SRCPMessageID, SRCPMessageType},
};

/// Auf dem Raspberry PI ab V2 werden folgende Ports verwendet:
/// - CTS GPIO3 (=Pin5)
/// - RTS GPIO27 (=Pin13)
/// - DTR GPIO4 (=Pin7)
/// - DSR GPIO2 (=Pin3)
const CTS: u32 = 3;
const RTS: u32 = 27;
const DTR: u32 = 4;
const DSR: u32 = 2;
/// Dauer Start- Stop Impuls siggmode
const DAUER_STOP_IMPULS_SIGG_MODE: Duration = Duration::from_millis(500);
const DAUER_START_IMPULS_SIGG_MODE: Duration = Duration::from_millis(750);
/// Verzögerung Power On Meldung um Booster allen Dekoder Zeit zum starten zu geben
const DELAY_POWER_ON_MELDUNG: Duration = Duration::from_millis(100);
/// Leitungen zum Booster ON ist 0 wegen Invertierung durch RS232 Treiber 0V->12V / 3.3V->-12V
const RS232_ON: u8 = 0;
const RS232_OFF: u8 = 1;
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
  //Zeit, die Booster On sein muss in uSekunden, damit bei Off (wegen Schluss) automatisch wieder eingeschaltet wird wenn Siggmode
  //0 = Ausgeschaltet, keine automatische Wiedereinschaltung.
  timeout_shortcut_power_off: Duration,
  //Aktueller Power Zustand
  power_on: bool,
  //Zeitpunkt Power On um On-Meldung verzögert zu liefern. Damit alle Dekoder Zeit haben zu starten.
  power_on_zeitpunkt: Instant,
  //Zeitpunkt Start/Stopimpulse wieder ausschalten siggmode
  impuls_aus: Instant,
  //Letzter Zeitpunkt Booster OK (kein Kurzsschluss) bei nicht siggmode
  kein_shortcut: Instant,
  //Bei Siggmode: Zeitpunkt, ab dem eine automatische Wiedereinschaltung erlaubt ist
  sigg_mode_auto_power_on: Option<Instant>,
  //Booster Go Meldung siggmode
  gpio_cts_go_in: LineHandle,
  //Booster Go Meldung / Shortcut
  gpio_dsr_go_in: LineHandle,
  //Booster Go Ausgang (Impuls bei siggmode)
  gpio_rts_go_out: LineHandle,
  //Booster Stop Ausgang Impuls bei siggmode
  gpio_dtr_stop_out: LineHandle,
}
impl DdlPower {
  /// Neue Instanz erstellen
  /// # Arguments
  /// * bus - SRCP Bus auf dem dieses Device arbeitet
  /// * tx - Sender für Info Messages / Antworten an SRCP Clients
  /// * siggmode - Impulse für Booster Start/Stop
  /// * dsr_invers - Inverse Behandlung DSR Booster Shortcut Rückmeldung wenn nicht siggmode
  /// * shortcut_delay - Verzögerung in ms bis Abschaltung wegen Shortcut wenn nicht siggmode
  /// * timeout_shortcut_power_off - Wenn Siggmode: minimale Power On Zeit damit einmalig bei Ausschaltung
  ///                                (wegen Kurzschluss) wieder versucht wird einzuschalten.
  ///                                0 = Ausgeschaltet, keine automatische Wiedereinschaltung.
  pub fn new(
    bus: usize, tx: Sender<SRCPMessage>, siggmode: bool, dsr_invers: bool, shortcut_delay: u64,
    timeout_shortcut_power_off: u64,
  ) -> DdlPower {
    let mut chip =
      Chip::new("/dev/gpiochip0").expect("/dev/gpiochip0 konnte nicht geöffnet werden");
    let result = DdlPower {
      bus: bus,
      tx: tx,
      siggmode: siggmode,
      dsr_invers: dsr_invers,
      shortcut_delay: Duration::from_millis(shortcut_delay),
      timeout_shortcut_power_off: Duration::from_millis(timeout_shortcut_power_off),
      power_on: false,
      power_on_zeitpunkt: Instant::now(),
      impuls_aus: Instant::now(),
      kein_shortcut: Instant::now(),
      sigg_mode_auto_power_on: None,
      gpio_cts_go_in: chip
        .get_line(CTS)
        .expect(format!("GPIO {} konnte nicht geöffnet werden", CTS).as_str())
        .request(LineRequestFlags::INPUT, 0, "input_cts_booster_go")
        .expect(format!("GPIO {} konnte nicht als Input geöffnet werden", CTS).as_str()),
      gpio_dsr_go_in: chip
        .get_line(DSR)
        .expect(format!("GPIO {} konnte nicht geöffnet werden", DSR).as_str())
        .request(LineRequestFlags::INPUT, 0, "input_dsr_booster_go")
        .expect(format!("GPIO {} konnte nicht als Input geöffnet werden", DSR).as_str()),
      gpio_rts_go_out: chip
        .get_line(RTS)
        .expect(format!("GPIO {} konnte nicht geöffnet werden", RTS).as_str())
        .request(LineRequestFlags::OUTPUT, 0, "output_rts_booster_go")
        .expect(format!("GPIO {} konnte nicht als Input geöffnet werden", RTS).as_str()),
      gpio_dtr_stop_out: chip
        .get_line(DTR)
        .expect(format!("GPIO {} konnte nicht geöffnet werden", DTR).as_str())
        .request(LineRequestFlags::OUTPUT, 0, "output_dtr_booster_stop")
        .expect(format!("GPIO {} konnte nicht als Input geöffnet werden", DTR).as_str()),
    };
    log::debug!(
      "New DdlPower siggmode={}, dsr_invers={}, shortcut_delay={}, timeout_shortcut_power_off={}",
      siggmode,
      dsr_invers,
      shortcut_delay,
      timeout_shortcut_power_off
    );
    //Default setzen, Ausgänge ausgeschaltet
    result.gpio_rts_go_out.set_value(RS232_OFF).unwrap();
    result.gpio_dtr_stop_out.set_value(RS232_OFF).unwrap();
    result
  }
  /// Siggmode Start-Stopimpulsausgabe
  /// # Arguments
  /// * power - true: Startimpuls, false: Stopimpuls
  fn start_stop_impuls(&mut self, power: bool) {
    if power {
      //Startimpuls
      self.gpio_rts_go_out.set_value(RS232_ON).unwrap();
      self.gpio_dtr_stop_out.set_value(RS232_OFF).unwrap();
      self.impuls_aus = Instant::now() + DAUER_START_IMPULS_SIGG_MODE;
      //Wenn Timeout für auto Wiedereinschaltung vorhanden ist
      if !self.timeout_shortcut_power_off.is_zero() {
        self.sigg_mode_auto_power_on = Some(Instant::now() + self.timeout_shortcut_power_off);
      }
    } else {
      //Stopimpuls
      self.gpio_dtr_stop_out.set_value(RS232_ON).unwrap();
      self.gpio_rts_go_out.set_value(RS232_OFF).unwrap();
      self.impuls_aus = Instant::now() + DAUER_STOP_IMPULS_SIGG_MODE;
    }
  }

  /// Neuer Power Zustand übernehmen
  /// # Arguments
  /// * power - Neuer Power Zustand
  fn set_power(&mut self, power: bool) {
    if self.power_on != power {
      self.power_on = power;
      self.send_all_info(None);
      if self.siggmode {
        self.start_stop_impuls(power);
      } else {
        //Booster On mit Dauerausgabe an RTS
        self
          .gpio_rts_go_out
          .set_value(if power { RS232_ON } else { RS232_OFF })
          .unwrap();
      }
      if power {
        //Soeben neu eingeschaltet.
        //Damit der Booster etwas Zeit hat um einzuschalten und erste Kommandos erst dann ausgegeben werden,
        //wenn sicher alle Dekoder gestartet haben -> Einschaltzeit merken. Power On wird erst mit Verzögerung gemeldet.
        self.power_on_zeitpunkt = Instant::now();
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
  /// * power - true wenn Power eingeschaltet, Booster On sind, hier nicht verwendet
  fn execute_cmd(&mut self, cmd_msg: &SRCPMessage, _power: bool) {
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
  /// Liefert hier den Power Zustand.
  /// Power On wird immer erst verzögert geliefert damit alle Dekoder aufstarten können, bevor erste Kommandoausgabe erfolgt.
  fn is_dev_spezifisch(&self) -> bool {
    self.power_on && (self.power_on_zeitpunkt + DELAY_POWER_ON_MELDUNG <= Instant::now())
  }

  /// Hintergrundaktivität:
  /// - Ausschalten Start- Stopimpulse zu Booster wenn siggmode
  /// - Kontrolle Boosterrückmeldung On/Off (Shortcut)
  /// Liefert immer false zurück, es wird hier nie ein Telegramm gesendet.
  /// # Arguments
  /// * power - true: Power / Booster ist ein, Strom auf den Schienen
  ///           false: Power / Booster ist aus
  ///           -> wird hier nicht verwendet, wir sind ja im DDL Device "Power"
  fn execute(&mut self, _power: bool) -> bool {
    if self.siggmode {
      //Wenn Start- Stop Impuls vorbei sind
      if Instant::now() > self.impuls_aus {
        //Start- Stop Impulse aus
        self.gpio_rts_go_out.set_value(RS232_OFF).unwrap();
        self.gpio_dtr_stop_out.set_value(RS232_OFF).unwrap();
        //Booster aus Erkennung nach Impulsausgabe
        let mut booster_on = self.gpio_cts_go_in.get_value().unwrap() == RS232_ON;
        //Wenn Timeout für auto Wiedereinschaltung vorhanden ist
        if let Some(sigg_mode_auto_power_on_zeitpunkt) = self.sigg_mode_auto_power_on {
          //Wenn nun Booster aus ist aber ein sein müsste und Timeout für automatische Wiedereinschaltung erreicht ist
          //-> automatischer Wiedereinschaltversuch
          if (!booster_on)
            && self.is_dev_spezifisch()
            && (sigg_mode_auto_power_on_zeitpunkt <= Instant::now())
          {
            self.start_stop_impuls(true);
            booster_on = true;
          }
        }
        //Aus- und Einschalten vom Booster übernehmen
        self.set_power(booster_on);
      }
    } else {
      //Kurzschluss- Erkennung
      let booster_on = (self.gpio_dsr_go_in.get_value().unwrap() == RS232_ON) ^ self.dsr_invers;
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
    false
  }
}
