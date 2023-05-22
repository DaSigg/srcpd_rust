use std::{
  thread,
  time::{Duration, Instant},
};

use log::info;
use spidev::{Spidev, SpidevTransfer};

use crate::{srcp_protocol_ddl::DdlTel, srcp_server_types::SRCPMessage};

/// Letzte Pause nach einem Telegramme. Gilt für alle Instanzen "SRCPDeviceDDL".
static mut LETZTE_PAUSE: Duration = Duration::ZERO;

/// Schnittstelle für alle Devices die in einem SRCP DDL Server bearbeitet werden
pub trait SRCPDeviceDDL {
  /// Empfangenes Kommando validieren.
  /// Return true wenn Kommando Ok.
  /// Sendet die Antwort Message (Ok / Err) an Sender zurück.
  /// # Arguments
  /// * cmd_msg - Empfangenes Kommando
  fn validate_cmd(&self, cmd_msg: &SRCPMessage) -> bool;
  /// Empfangenes Kommando ausführen und versenden, ggf. interne Daten Updaten für späteren Refresh.
  /// Das Kommando muss gültig sein (validate_cmd), es wird hier nicht ,ehr überprüft.
  /// # Arguments
  /// * cmd_msg - Empfangenes Kommando
  fn execute_cmd(&mut self, cmd_msg: &SRCPMessage);
  /// Refresh Zyklus Telegramm senden (wird nur für GL aufgerufen)
  fn send_refresh(&mut self) {}
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
  /// Das gesendete Teleramm wird aus "ddl_tel" gelöscht.
  /// * spidev - Geöffnetes SPI interface über das Telegramme zum Booster gesendet werden können
  /// * ddl_tel - Das zu sendende Telegramm. Es wird hier nur das erste Teleramm gesendet und dann gelöscht.
  fn send(spidev: &Option<Spidev>, ddl_tel: &mut DdlTel)
  where
    Self: Sized,
  {
    assert!(
      ddl_tel.daten.len() > 0,
      "Aufruf SRCPDeviceDDL::send mit leerem ddl_tel"
    );
    let mut pause = Duration::ZERO;
    unsafe {
      //Zugriff erfolgt nur aus einem Thread, also OK
      if LETZTE_PAUSE < ddl_tel.pause_start {
        //Es ist noch eine Pause zum Start notwendig
        pause = ddl_tel.pause_start - LETZTE_PAUSE;
      }
    }
    if !pause.is_zero() {
      thread::sleep(pause);
    }
    if ddl_tel.adr == 40 {
      info!("Send DDL Adr={}", ddl_tel.adr);
    }
    let mut transfer = SpidevTransfer::write(ddl_tel.daten[0].as_slice());
    transfer.speed_hz = ddl_tel.hz;
    spidev
      .as_ref()
      .unwrap()
      .transfer(&mut transfer)
      .expect("DDL SPI write fail");
    //Und jetzt löschen was gesendet wurde
    ddl_tel.daten.remove(0);
    //Wann darf das nächste Telegramm (wenn vorhanden) gesendet werden
    ddl_tel.instant_next = Some(Instant::now() + ddl_tel.delay);
    unsafe {
      //Zugriff erfolgt nur aus einem Thread, also OK
      LETZTE_PAUSE = ddl_tel.pause_ende;
    }
  }
}
