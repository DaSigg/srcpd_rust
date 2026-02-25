use std::{
  thread,
  time::{Duration, Instant},
};

use gpio_cdev::{Chip, LineHandle, LineRequestFlags};
use log::warn;
use spidev::{Spidev, SpidevTransfer};

use crate::{
  srcp_protocol_ddl::DdlTel, srcp_protocol_ddl::DdlTelRx, srcp_server_types::SRCPMessage,
};

/// Schnittstelle für alle Devices die in einem SRCP DDL Server bearbeitet werden
pub trait SRCPDeviceDDL {
  /// Empfangenes Kommando validieren.
  /// Return true wenn Kommando Ok.
  /// Sendet die Antwort Message (Ok / Err) an Sender zurück.
  /// # Arguments
  /// * cmd_msg - Empfangenes Kommando
  fn validate_cmd(&self, cmd_msg: &SRCPMessage) -> bool;
  /// Empfangenes Kommando ausführen und versenden, ggf. interne Daten Updaten für späteren Refresh.
  /// Das Kommando muss gültig sein (validate_cmd), es wird hier nicht mehr überprüft.
  /// # Arguments
  /// * cmd_msg - Empfangenes Kommando
  /// * power - true wenn Power eingeschaltet, Booster On sind
  fn execute_cmd(&mut self, cmd_msg: &SRCPMessage, power: bool);
  /// Refresh Zyklus Telegramm senden (wird nur für GL aufgerufen)
  fn send_refresh(&mut self) {}
  /// Muss zyklisch aufgerufen werden. Erlaubt dem Device die Ausführung von
  /// von neuen Kommando oder refresh unabhängigen Aufgaben.
  /// Liefert true zurück, wenn durch den Aufruf min. ein DDL Telegramm gesendet wurde, sonst false.
  /// # Arguments
  /// * power - true: Power / Booster ist ein, Strom auf den Schienen
  ///           false: Power / Booster ist aus
  fn execute(&mut self, _power: bool) -> bool {
    false
  }
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
  /// Das gesendete Teleramm wird aus "ddl_tel" gelöscht.
  /// # Arguments
  /// * spidev - Geöffnetes SPI interface über das Telegramme zum Booster gesendet werden können
  /// * ddl_tel - Das zu sendende Telegramm. Es wird hier nur das erste Teleramm gesendet und dann gelöscht.
  /// * trigger_port - Oszi trigger Port aus Konfigfile
  fn send(spidev: &Option<Spidev>, ddl_tel: &mut DdlTel, trigger_port: Option<u32>)
  where
    Self: Sized,
  {
    assert!(
      ddl_tel.daten.len() > 0,
      "Aufruf SRCPDeviceDDL::send mit leerem ddl_tel"
    );
    //Debug Oszi Trigger
    let mut gpio_trigger_out: Option<LineHandle> = None;
    if ddl_tel.trigger && trigger_port.is_some() {
      gpio_trigger_out = Some(
        Chip::new("/dev/gpiochip0")
          .expect("/dev/gpiochip0 konnte nicht geöffnet werden")
          .get_line(trigger_port.unwrap())
          .expect("GPIO für Oszi Trigger konnte nicht geöffnet werden")
          .request(LineRequestFlags::OUTPUT, 1, "output_trigger_ddl")
          .expect("GPIO für Oszi Trigger konnte nicht als Output geöffnet werden"),
      );
    }
    //Verlangte Pause nach Telegramme des letzten gesendten Telegrammes. Gilt für alle Instanzen "SRCPDeviceDDL".
    static mut LETZTE_PAUSE_ENDE: Duration = Duration::ZERO;
    //Zugriff erfolgt nur aus einem Thread, also OK
    unsafe {
      if LETZTE_PAUSE_ENDE > ddl_tel.pause_start {
        //Es ist noch eine Pause zum Start aufgrund verlangter Pause am Ende des letztens Telegrammes notwendig
        thread::sleep(LETZTE_PAUSE_ENDE - ddl_tel.pause_start);
      }
      LETZTE_PAUSE_ENDE = ddl_tel.pause_ende;
    }

    let mut transfer = match ddl_tel.daten_rx {
      DdlTelRx::SpiRx(ref mut daten_rx) if ddl_tel.daten.len() == 1 => {
        assert_eq!(
          ddl_tel.daten[0].len(),
          daten_rx.len(),
          "Bei Verwendung DdlTel::daten_rx muss dessen Länge gleich wie letztes gesendetes Tel sein."
        );
        SpidevTransfer::read_write(ddl_tel.daten[0].as_slice(), daten_rx.as_mut_slice())
      }
      DdlTelRx::SpiRx(_) | DdlTelRx::None | DdlTelRx::Udp => {
        SpidevTransfer::write(ddl_tel.daten[0].as_slice())
      }
    };
    transfer.speed_hz = ddl_tel.hz;
    for _ in 0..ddl_tel.tel_wiederholungen {
      spidev
        .as_ref()
        .unwrap()
        .transfer(&mut transfer)
        .expect("DDL SPI write fail");
    }
    //Oszi Trigger zurücknehmen wenn ausgegeben
    if gpio_trigger_out.is_some() {
      gpio_trigger_out.unwrap().set_value(0).unwrap();
    }
    //Und jetzt löschen was gesendet wurde
    ddl_tel.daten.remove(0);
    //Wann darf das nächste Telegramm (wenn vorhanden) gesendet werden
    ddl_tel.instant_next = Some(Instant::now() + ddl_tel.delay);
  }

  /// Auswerten Oszi Trigger Konfiguration.
  /// Liefert Oszi Triggerport zurück.
  /// # Arguments
  /// * port - Port als String aus Konfigfile, None wenn nicht vorhanden
  fn eval_trigger_port_config(&self, port: Option<String>) -> Option<u32> {
    if let Some(p) = port {
      if let Ok(port_nr) = p.parse::<u32>() {
        return Some(port_nr);
      } else {
        warn!("DDL: Ungültiger Oszi Triggerport: {}", p);
      }
    }
    return None;
  }

  /// Auswerten Oszi Trigger Konfiguration.
  /// Liefert Oszi Trigger Vector mit allen Adressen für Triggerausgabe zurück
  /// # Arguments
  /// * adressen - Liste mit Adressen für Oszi Trigger (gtrennt mit Kommas) aus Konfigfile
  fn eval_trigger_config(&self, adressen: Option<String>) -> Vec<u32> {
    let mut result_adressen: Vec<u32> = vec![];
    if let Some(adr) = adressen {
      for adresse_str in adr.split(",") {
        if let Ok(adresse) = adresse_str.parse::<u32>() {
          result_adressen.push(adresse);
        } else {
          warn!("DDL: Ungültiger Oszi Triggeradresse: {}", adresse_str);
        }
      }
    }
    return result_adressen;
  }
}
